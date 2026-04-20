"""Estimate per-joint contact level from motor current + fingertip F/T.

Subscribes to:
  * sensor_msgs/JointState    (motor current in the effort field;
                               scale varies, reaches 100+ on firm grip)
  * 5 x geometry_msgs/WrenchStamped from the fingertip broadcasters
    (/dg5f_right/fingertip_{1..5}_broadcaster/wrench)

Publishes two std_msgs/Float32MultiArray topics:
  * /dg5f_right/contact_level           — 20 values (4 joints x 5 fingers)
  * /dg5f_right/fingertip_contact_level —  5 values (one per fingertip)

Joint pipeline (per joint):
  raw effort + velocity
    -> if |velocity| > motion_velocity_threshold:
         reset stability, CLEAR median history (stale samples are not
         safe post-motion), publish 0 for that joint
    -> else: median-window -> subtract baseline -> normalize [0,1]

Motion gating is the dominant filter for Manus teleop, where the hand
is almost always moving. Clearing the history on every motion frame
ensures post-motion readings aren't contaminated by pre-motion values.

Fingertip pipeline (per finger):
  |F| = sqrt(fx^2 + fy^2 + fz^2)
    -> EMA smooth -> normalize via fingertip_force_low / high
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped

DG5F_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

DEFAULT_FINGERTIP_TOPICS = [
    f"/dg5f_right/fingertip_{i}_broadcaster/wrench" for i in range(1, 6)
]


class ContactMonitor(Node):
    def __init__(self):
        super().__init__("contact_monitor")

        # ---- joint-side parameters ----
        self.declare_parameter("joint_states_topic", "/dg5f_right/joint_states")
        self.declare_parameter("contact_level_topic", "/dg5f_right/contact_level")
        self.declare_parameter("contact_low", [2.0] * 20)
        self.declare_parameter("contact_high", [8.0] * 20)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("median_window", 15)
        self.declare_parameter("reject_above", 1000.0)
        self.declare_parameter("baseline_samples", 60)
        self.declare_parameter("motion_velocity_threshold", 0.05)
        self.declare_parameter("motion_settle_samples", 10)

        # ---- fingertip F/T parameters ----
        self.declare_parameter("fingertip_level_topic",
                               "/dg5f_right/fingertip_contact_level")
        self.declare_parameter("fingertip_topics", DEFAULT_FINGERTIP_TOPICS)
        self.declare_parameter("fingertip_force_low", 0.5)   # N
        self.declare_parameter("fingertip_force_high", 5.0)  # N
        self.declare_parameter("fingertip_ema_alpha", 0.3)   # EMA smoothing

        self._in_topic = self.get_parameter("joint_states_topic").value
        self._out_topic = self.get_parameter("contact_level_topic").value
        self._low = list(self.get_parameter("contact_low").value)
        self._high = list(self.get_parameter("contact_high").value)
        self._rate = float(self.get_parameter("publish_rate_hz").value)
        self._win_size = max(1, int(self.get_parameter("median_window").value))
        self._reject_above = float(self.get_parameter("reject_above").value)
        self._baseline_n = max(1, int(self.get_parameter("baseline_samples").value))
        self._motion_vel_thr = float(
            self.get_parameter("motion_velocity_threshold").value)
        self._motion_settle_n = max(
            1, int(self.get_parameter("motion_settle_samples").value))

        self._tip_out_topic = self.get_parameter("fingertip_level_topic").value
        self._tip_topics = list(self.get_parameter("fingertip_topics").value)
        self._tip_low = float(self.get_parameter("fingertip_force_low").value)
        self._tip_high = float(self.get_parameter("fingertip_force_high").value)
        self._tip_alpha = float(self.get_parameter("fingertip_ema_alpha").value)

        if len(self._low) != 20 or len(self._high) != 20:
            raise ValueError("contact_low / contact_high must have 20 entries")

        # ---- joint state ----
        self._history = [deque(maxlen=self._win_size) for _ in range(20)]
        self._baseline_buf = [deque(maxlen=self._baseline_n) for _ in range(20)]
        self._baseline = [0.0] * 20
        self._baseline_ready = False
        self._calibrating = True
        self._latest = [0.0] * 20
        self._motion_stable = [0] * 20
        self._have_data = False

        # ---- fingertip state ----
        self._tip_force = [0.0] * 5            # EMA-smoothed magnitude (N)
        self._tip_have_any = False

        # ---- ROS plumbing ----
        self._sub = self.create_subscription(
            JointState, self._in_topic, self._on_js, 10)
        self._pub = self.create_publisher(
            Float32MultiArray, self._out_topic, 10)
        self._tip_pub = self.create_publisher(
            Float32MultiArray, self._tip_out_topic, 10)
        self._zero_srv = self.create_service(
            Trigger, "~/zero_baseline", self._zero_cb)

        self._tip_subs = []
        for idx, topic in enumerate(self._tip_topics[:5]):
            # Use an ordinary lambda default arg to bind idx.
            cb = (lambda msg, i=idx: self._on_wrench(i, msg))
            self._tip_subs.append(self.create_subscription(
                WrenchStamped, topic, cb, 10))

        period = 1.0 / max(self._rate, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"contact_monitor: {self._in_topic} -> {self._out_topic} "
            f"@ {self._rate:.1f} Hz | median={self._win_size} "
            f"| motion_gate |v|<{self._motion_vel_thr:.3f} rad/s "
            f"for {self._motion_settle_n} samples "
            f"| calibrating baseline for {self._baseline_n} samples... "
            f"(HOLD THE HAND STILL)"
        )
        self.get_logger().info(
            f"fingertip: {len(self._tip_subs)} wrench subs -> {self._tip_out_topic} "
            f"| force thresholds {self._tip_low:.2f}..{self._tip_high:.2f} N"
        )

    # ---------------- subscribers -----------------

    def _on_js(self, msg: JointState):
        if not msg.effort:
            return
        name_to_eff = dict(zip(msg.name, msg.effort))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        for i, j in enumerate(DG5F_JOINTS):
            if j not in name_to_eff:
                continue
            raw = abs(float(name_to_eff[j]))
            if raw > self._reject_above:
                continue
            vel = abs(float(name_to_vel.get(j, 0.0)))

            if vel > self._motion_vel_thr:
                # Motion frame: don't contaminate median/baseline. Also clear
                # history — stale pre-motion samples are not representative
                # of the current state once motion has begun.
                self._motion_stable[i] = 0
                self._history[i].clear()
                continue

            self._motion_stable[i] = min(self._motion_stable[i] + 1,
                                         self._motion_settle_n + 1)
            self._history[i].append(raw)
            med = sorted(self._history[i])[len(self._history[i]) // 2]
            if self._calibrating:
                self._baseline_buf[i].append(med)
            self._latest[i] = med

        self._have_data = True

        if self._calibrating and all(len(b) >= self._baseline_n
                                     for b in self._baseline_buf):
            self._finish_calibration()

    def _on_wrench(self, idx: int, msg: WrenchStamped):
        fx = float(msg.wrench.force.x)
        fy = float(msg.wrench.force.y)
        fz = float(msg.wrench.force.z)
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        # EMA smoothing
        a = self._tip_alpha
        self._tip_force[idx] = a * mag + (1.0 - a) * self._tip_force[idx]
        self._tip_have_any = True

    # ---------------- service --------------------

    def _zero_cb(self, request, response):
        self.get_logger().info("zero_baseline requested — recalibrating")
        for b in self._baseline_buf:
            b.clear()
        for h in self._history:
            h.clear()
        for i in range(20):
            self._motion_stable[i] = 0
        self._baseline_ready = False
        self._calibrating = True
        response.success = True
        response.message = "recalibrating baseline"
        return response

    def _finish_calibration(self):
        for i in range(20):
            buf = list(self._baseline_buf[i])
            buf.sort()
            self._baseline[i] = buf[len(buf) // 2]
        self._baseline_ready = True
        self._calibrating = False
        short = ", ".join(f"{b:.1f}" for b in self._baseline)
        self.get_logger().info(f"baseline calibrated: [{short}]")

    # ---------------- publisher ------------------

    def _tick(self):
        if not self._have_data:
            return
        # Joint contact (20)
        out = Float32MultiArray()
        out.layout.dim = [MultiArrayDimension(label="joint", size=20, stride=20)]
        if self._calibrating or not self._baseline_ready:
            out.data = [0.0] * 20
        else:
            out.data = [
                self._normalize(i, self._latest[i])
                if self._motion_stable[i] >= self._motion_settle_n else 0.0
                for i in range(20)
            ]
        self._pub.publish(out)

        # Fingertip F/T (5)
        tip = Float32MultiArray()
        tip.layout.dim = [MultiArrayDimension(label="finger", size=5, stride=5)]
        tip.data = [self._normalize_tip(i) for i in range(5)]
        self._tip_pub.publish(tip)

    def _normalize(self, i: int, raw: float) -> float:
        adjusted = raw - self._baseline[i]
        if adjusted < 0.0:
            adjusted = 0.0
        lo, hi = self._low[i], self._high[i]
        if hi <= lo:
            return 0.0
        x = (adjusted - lo) / (hi - lo)
        return float(max(0.0, min(1.0, x)))

    def _normalize_tip(self, i: int) -> float:
        if self._tip_high <= self._tip_low:
            return 0.0
        x = (self._tip_force[i] - self._tip_low) / (self._tip_high - self._tip_low)
        return float(max(0.0, min(1.0, x)))


def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
