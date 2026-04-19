"""Estimate per-joint contact level from motor-current feedback.

Subscribes to sensor_msgs/JointState (effort field = motor current in mA in the
DG5F stack) and publishes a normalized 20-float contact level on
std_msgs/Float32MultiArray, where 0.0 = no contact (green) and 1.0 = hard
contact (red). Layout matches the 20 DG5F joints in the order they appear on
/joint_states.
"""

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
        self.declare_parameter("contact_low_mA", [50.0] * 20)
        self.declare_parameter("contact_high_mA", [400.0] * 20)
        self.declare_parameter("publish_rate_hz", 30.0)

        self._in_topic = self.get_parameter("joint_states_topic").value
        self._out_topic = self.get_parameter("contact_level_topic").value
        self._low = list(self.get_parameter("contact_low_mA").value)
        self._high = list(self.get_parameter("contact_high_mA").value)
        self._rate = float(self.get_parameter("publish_rate_hz").value)

        if len(self._low) != 20 or len(self._high) != 20:
            raise ValueError("contact_low_mA / contact_high_mA must have 20 entries")

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
            f"@ {self._rate:.1f} Hz"
        )

    def _on_js(self, msg: JointState):
        if not msg.effort:
            return
        name_to_eff = dict(zip(msg.name, msg.effort))
        for i, j in enumerate(DG5F_JOINTS):
            if j in name_to_eff:
                self._latest[i] = abs(float(name_to_eff[j]))
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
