"""Estimate per-joint contact level from motor-current feedback.

Subscribes to sensor_msgs/JointState (the effort field carries motor
current on the DG5F stack; scale varies by firmware, reaches 100+ on
firm grip) and publishes a normalized 20-float contact level on
std_msgs/Float32MultiArray, where 0.0 = no contact (green) and 1.0 =
hard contact (red).

Joint pipeline (per finger):
  * read effort & velocity for all 4 joints of the finger
  * if the maximum |velocity| across the finger's joints exceeds
    motion_velocity_threshold -> finger is moving. Reset that finger's
    stability counter and publish 0s for all of this finger's joints
    this tick. This covers tendon coupling: J4 current rises while J2
    is flexing even though J4's own velocity is near zero, so gating
    per-joint leaves a residual flicker. Finger-scope gating does not.
    The median history is NOT cleared — old samples age out via the
    rolling window, preventing single-sample bursts from flashing the
    lamp right after motion stops.
  * otherwise median-window -> subtract baseline -> normalize [0,1]

Motion gating is the dominant filter for Manus teleop, where the hand
is almost always moving.

The baseline step handles per-joint idle bias: distal joints carry
gravity load, so their idle current is not the same as the base
joints. Subtracting the observed idle gives every joint a clean 0
reference.
"""

from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

DG5F_JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]


class ContactMonitor(Node):
    def __init__(self):
        super().__init__("contact_monitor")

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

        if len(self._low) != 20 or len(self._high) != 20:
            raise ValueError("contact_low / contact_high must have 20 entries")

        self._history = [deque(maxlen=self._win_size) for _ in range(20)]
        self._baseline_buf = [deque(maxlen=self._baseline_n) for _ in range(20)]
        self._baseline = [0.0] * 20
        self._baseline_ready = False
        self._calibrating = True
        self._latest = [0.0] * 20
        self._finger_stable = [0] * 5
        self._have_data = False

        self._sub = self.create_subscription(
            JointState, self._in_topic, self._on_js, 10)
        self._pub = self.create_publisher(
            Float32MultiArray, self._out_topic, 10)
        self._zero_srv = self.create_service(
            Trigger, "~/zero_baseline", self._zero_cb)

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

    # ---------------- subscriber -----------------

    def _on_js(self, msg: JointState):
        if not msg.effort:
            return
        name_to_eff = dict(zip(msg.name, msg.effort))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}

        for f in range(5):
            joint_idxs = [f * 4 + k for k in range(4)]
            names = [DG5F_JOINTS[i] for i in joint_idxs]
            if not all(n in name_to_eff for n in names):
                continue

            max_vel = max(abs(float(name_to_vel.get(n, 0.0))) for n in names)
            if max_vel > self._motion_vel_thr:
                self._finger_stable[f] = 0
                continue

            self._finger_stable[f] = min(self._finger_stable[f] + 1,
                                         self._motion_settle_n + 1)
            for i, n in zip(joint_idxs, names):
                raw = abs(float(name_to_eff[n]))
                if raw > self._reject_above:
                    continue
                self._history[i].append(raw)
                med = sorted(self._history[i])[len(self._history[i]) // 2]
                if self._calibrating:
                    self._baseline_buf[i].append(med)
                self._latest[i] = med

        self._have_data = True

        if self._calibrating and all(len(b) >= self._baseline_n
                                     for b in self._baseline_buf):
            self._finish_calibration()

    # ---------------- service --------------------

    def _zero_cb(self, request, response):
        self.get_logger().info("zero_baseline requested — recalibrating")
        for b in self._baseline_buf:
            b.clear()
        for h in self._history:
            h.clear()
        for f in range(5):
            self._finger_stable[f] = 0
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
        out = Float32MultiArray()
        out.layout.dim = [MultiArrayDimension(label="joint", size=20, stride=20)]
        if self._calibrating or not self._baseline_ready:
            out.data = [0.0] * 20
        else:
            data = []
            for f in range(5):
                active = self._finger_stable[f] >= self._motion_settle_n
                for k in range(4):
                    i = f * 4 + k
                    data.append(self._normalize(i, self._latest[i])
                                if active else 0.0)
            out.data = data
        self._pub.publish(out)

    def _normalize(self, i: int, raw: float) -> float:
        adjusted = raw - self._baseline[i]
        if adjusted < 0.0:
            adjusted = 0.0
        lo, hi = self._low[i], self._high[i]
        if hi <= lo:
            return 0.0
        x = (adjusted - lo) / (hi - lo)
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
