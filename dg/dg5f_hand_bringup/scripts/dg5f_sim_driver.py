#!/usr/bin/env python3
"""Fake DG5F driver for pre-hardware testing.

Publishes /dg5f_right/joint_states at a configurable rate with synthetic
position/velocity/effort so the downstream visualization stack can be
exercised end-to-end without a physical gripper.

Scenarios (parameter `scenario`):
  * idle       — all joints at 0, no effort
  * sweep      — each joint follows a sinusoid; efforts oscillate between
                 `base_current_mA` and `peak_current_mA` at different phases
  * grip_cycle — close (effort ramps up) → contact hold on a configurable
                 set of joints → release → repeat
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]


class SimDriver(Node):
    def __init__(self):
        super().__init__("dg5f_sim_driver")

        self.declare_parameter("topic", "/dg5f_right/joint_states")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("scenario", "grip_cycle")  # idle | sweep | grip_cycle
        self.declare_parameter("base_current_mA", 20.0)
        self.declare_parameter("peak_current_mA", 550.0)
        self.declare_parameter("cycle_period_s", 6.0)
        # Joints that "grip an object" in grip_cycle (distal joints by default).
        self.declare_parameter("contact_joint_indices",
                               [3, 7, 11, 15, 19])  # F?_J4 (last joint of each finger)

        self._topic = self.get_parameter("topic").value
        self._rate = float(self.get_parameter("rate_hz").value)
        self._scenario = str(self.get_parameter("scenario").value)
        self._base = float(self.get_parameter("base_current_mA").value)
        self._peak = float(self.get_parameter("peak_current_mA").value)
        self._period = max(0.5, float(self.get_parameter("cycle_period_s").value))
        self._contact_idx = set(int(i) for i in
                                self.get_parameter("contact_joint_indices").value)

        self._pub = self.create_publisher(JointState, self._topic, 10)
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"dg5f_sim_driver: {self._scenario} -> {self._topic} @ {self._rate:.0f} Hz"
        )

    def _tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self._t0

        pos, vel, eff = self._compute(t)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(JOINTS)
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self._pub.publish(msg)

    def _compute(self, t: float):
        n = len(JOINTS)
        pos = [0.0] * n
        vel = [0.0] * n
        eff = [0.0] * n

        if self._scenario == "idle":
            return pos, vel, eff

        if self._scenario == "sweep":
            for i in range(n):
                phase = 2.0 * math.pi * (t / self._period + i / n)
                s = 0.5 * (1.0 + math.sin(phase))  # [0, 1]
                pos[i] = 0.4 * math.sin(phase)
                vel[i] = 0.4 * (2.0 * math.pi / self._period) * math.cos(phase)
                eff[i] = self._base + (self._peak - self._base) * s
            return pos, vel, eff

        # grip_cycle: 4 phases per period — close, hold, open, rest.
        u = (t % self._period) / self._period  # [0, 1)
        close_env = 0.0
        hold = False
        if u < 0.25:
            close_env = u / 0.25                       # 0 -> 1
        elif u < 0.55:
            close_env = 1.0                            # hold
            hold = True
        elif u < 0.80:
            close_env = 1.0 - (u - 0.55) / 0.25        # 1 -> 0 (open)
        else:
            close_env = 0.0                            # rest

        for i in range(n):
            pos[i] = 0.9 * close_env
            vel[i] = 0.0  # simplified
            base = self._base + (self._peak - self._base) * 0.25 * close_env
            if hold and i in self._contact_idx:
                eff[i] = self._peak  # contact saturates
            else:
                eff[i] = base
        return pos, vel, eff


def main(args=None):
    rclpy.init(args=args)
    node = SimDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
