"""Estimate per-joint contact level from motor-current feedback.

Subscribes to sensor_msgs/JointState (effort field on the DG5F stack
carries motor current; scale varies by firmware, roughly 0.0–150+ on
this testbed) and publishes a normalized 20-float contact level on
std_msgs/Float32MultiArray, where 0.0 = no contact (green) and 1.0 =
hard contact (red).

A per-joint sliding-window median filter is applied before normalization
to reject occasional Modbus comms glitches (isolated values like 152 or
550 appearing on otherwise-quiet joints).
"""

from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

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
        self.declare_parameter("median_window", 5)        # 1 disables filter
        self.declare_parameter("reject_above", 1000.0)    # hard-reject obviously bogus samples

        self._in_topic = self.get_parameter("joint_states_topic").value
        self._out_topic = self.get_parameter("contact_level_topic").value
        self._low = list(self.get_parameter("contact_low").value)
        self._high = list(self.get_parameter("contact_high").value)
        self._rate = float(self.get_parameter("publish_rate_hz").value)
        self._win_size = max(1, int(self.get_parameter("median_window").value))
        self._reject_above = float(self.get_parameter("reject_above").value)

        if len(self._low) != 20 or len(self._high) != 20:
            raise ValueError("contact_low / contact_high must have 20 entries")

        self._history = [deque(maxlen=self._win_size) for _ in range(20)]
        self._latest = [0.0] * 20
        self._have_data = False

        self._sub = self.create_subscription(
            JointState, self._in_topic, self._on_js, 10
        )
        self._pub = self.create_publisher(
            Float32MultiArray, self._out_topic, 10
        )
        period = 1.0 / max(self._rate, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"contact_monitor: {self._in_topic} -> {self._out_topic} "
            f"@ {self._rate:.1f} Hz, median_window={self._win_size}, "
            f"reject_above={self._reject_above}"
        )

    def _on_js(self, msg: JointState):
        if not msg.effort:
            return
        name_to_eff = dict(zip(msg.name, msg.effort))
        for i, j in enumerate(DG5F_JOINTS):
            if j not in name_to_eff:
                continue
            raw = abs(float(name_to_eff[j]))
            # Obviously-bogus values (Modbus frame corruption): drop entirely.
            if raw > self._reject_above:
                continue
            hist = self._history[i]
            hist.append(raw)
            # Per-joint sliding-window median — rejects isolated spikes.
            self._latest[i] = sorted(hist)[len(hist) // 2]
        self._have_data = True

    def _tick(self):
        if not self._have_data:
            return
        out = Float32MultiArray()
        dim = MultiArrayDimension(label="joint", size=20, stride=20)
        out.layout.dim = [dim]
        out.data = [self._normalize(i, self._latest[i]) for i in range(20)]
        self._pub.publish(out)

    def _normalize(self, i: int, mA: float) -> float:
        lo, hi = self._low[i], self._high[i]
        if hi <= lo:
            return 0.0
        x = (mA - lo) / (hi - lo)
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
