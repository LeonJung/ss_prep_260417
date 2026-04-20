"""Estimate per-joint contact level from motor-current feedback.

Subscribes to sensor_msgs/JointState (effort field on the DG5F stack
carries motor current; scale varies by firmware, reaches 100+ on firm
contact on this testbed) and publishes a normalized 20-float contact
level on std_msgs/Float32MultiArray, where 0.0 = no contact (green)
and 1.0 = hard contact (red).

Pipeline per joint:
  raw effort + velocity
    -> if |velocity| > motion_velocity_threshold -> motion-gate:
         reset stability counter, drop sample, publish 0 (pure green)
    -> else
       hard reject anything above `reject_above`   (Modbus corruption)
       -> sliding-window median over `median_window` samples
       -> subtract per-joint baseline  (auto-calibrated at startup, or
                                        via /contact_monitor/zero_baseline)
       -> clamp >= 0
       -> linear map to [0, 1] using contact_low / contact_high

Motion gating: commanded finger motion draws non-trivial motor current
even when nothing is touched. Masking samples collected while the joint
is moving lets the lamp respond only to external resistance (contact,
obstruction) once the joint settles for motion_settle_samples frames.
This is critical for Manus teleop where the hand moves almost
continuously.

The baseline step handles per-joint idle bias: distal joints carry
gravity load, so their idle current is not the same as the base joints.
Subtracting the observed idle gives every joint a clean 0 reference.
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
        self.declare_parameter("baseline_samples", 60)        # ~2 s @ 30 Hz
        self.declare_parameter("motion_velocity_threshold", 0.05)  # rad/s
        self.declare_parameter("motion_settle_samples", 5)    # ~170 ms @ 30 Hz

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
        self._motion_stable = [0] * 20   # consecutive below-threshold samples
        self._have_data = False

        self._sub = self.create_subscription(
            JointState, self._in_topic, self._on_js, 10
        )
        self._pub = self.create_publisher(
            Float32MultiArray, self._out_topic, 10
        )
        self._zero_srv = self.create_service(
            Trigger, "~/zero_baseline", self._zero_cb
        )
        period = 1.0 / max(self._rate, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"contact_monitor: {self._in_topic} -> {self._out_topic} "
            f"@ {self._rate:.1f} Hz | median_window={self._win_size} "
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
        for i, j in enumerate(DG5F_JOINTS):
            if j not in name_to_eff:
                continue
            raw = abs(float(name_to_eff[j]))
            if raw > self._reject_above:
                continue
            vel = abs(float(name_to_vel.get(j, 0.0)))

            if vel > self._motion_vel_thr:
                # Joint is moving: don't contaminate the median window or the
                # baseline buffer. Reset stability so lamp stays dim until the
                # joint settles for motion_settle_n consecutive frames.
                self._motion_stable[i] = 0
                continue

            self._motion_stable[i] = min(self._motion_stable[i] + 1,
                                         self._motion_settle_n + 1)
            self._history[i].append(raw)
            med = sorted(self._history[i])[len(self._history[i]) // 2]
            if self._calibrating:
                self._baseline_buf[i].append(med)
            self._latest[i] = med

        self._have_data = True

        # Finish calibration once every joint has enough samples.
        if self._calibrating and all(len(b) >= self._baseline_n
                                     for b in self._baseline_buf):
            self._finish_calibration()

    # ---------------- service --------------------

    def _zero_cb(self, request, response):
        self.get_logger().info("zero_baseline requested — recalibrating")
        for b in self._baseline_buf:
            b.clear()
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
            # Use median over the calib window — robust to any stray glitches
            # that slipped past reject_above during calibration.
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
        dim = MultiArrayDimension(label="joint", size=20, stride=20)
        out.layout.dim = [dim]
        if self._calibrating or not self._baseline_ready:
            out.data = [0.0] * 20  # pure green while learning the floor
        else:
            out.data = [
                self._normalize(i, self._latest[i])
                if self._motion_stable[i] >= self._motion_settle_n else 0.0
                for i in range(20)
            ]
        self._pub.publish(out)

    def _normalize(self, i: int, raw: float) -> float:
        adjusted = raw - self._baseline[i]
        if adjusted < 0.0:
            adjusted = 0.0
        lo, hi = self._low[i], self._high[i]
        if hi <= lo:
            return 0.0
        x = (adjusted - lo) / (hi - lo)
        if x < 0.0:
            return 0.0
        if x > 1.0:
            return 1.0
        return float(x)


def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
