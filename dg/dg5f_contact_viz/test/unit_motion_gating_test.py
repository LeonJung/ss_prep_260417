#!/usr/bin/env python3
"""Unit test for motion-gated contact detection.

Exercises ContactMonitor's _on_js + _tick directly by feeding fabricated
JointState messages. No ROS router needed; uses rclpy.init + direct calls.

Scenarios:
  1. Effort 50, velocity 0.3 (motion) -> output 0 even after many frames
  2. Effort 50, velocity 0.0 for settle + window samples -> output > 0
  3. Mixed: motion briefly breaks stability -> lamp drops to 0
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


# Import from the installed package path.
from dg5f_contact_viz.contact_monitor_node import ContactMonitor, DG5F_JOINTS


class Collector(Node):
    def __init__(self):
        super().__init__("motion_gating_collector")
        self.samples = []
        self.create_subscription(
            Float32MultiArray, "/dg5f_right/contact_level",
            lambda m: self.samples.append(list(m.data)), 10,
        )


def _make_js(effort_val: float, vel_val: float) -> JointState:
    msg = JointState()
    msg.name = list(DG5F_JOINTS)
    msg.position = [0.0] * 20
    msg.velocity = [vel_val] * 20
    msg.effort = [effort_val] * 20
    return msg


def _drive(monitor: ContactMonitor, collector: Collector, msg: JointState,
           n: int):
    for _ in range(n):
        monitor._on_js(msg)
        monitor._tick()
        rclpy.spin_once(collector, timeout_sec=0.01)


def main():
    rclpy.init()

    # Short baseline so the test runs quickly.
    monitor = ContactMonitor()
    monitor._baseline_n = 5
    # Re-size buffers to match shortened baseline.
    from collections import deque
    monitor._baseline_buf = [deque(maxlen=5) for _ in range(20)]
    # Reduce motion settle count so we can complete the test quickly.
    monitor._motion_settle_n = 3
    monitor._motion_vel_thr = 0.05

    collector = Collector()

    # Step A: 10 stationary frames with low effort -> baseline calibrates.
    _drive(monitor, collector, _make_js(effort_val=2.0, vel_val=0.0), 10)
    assert monitor._baseline_ready, "baseline should be ready after 10 still frames"
    print(f"PASS baseline calibrated to {monitor._baseline[0]:.2f}")

    # Step B: now feed high effort + high velocity (motion).
    #         Expect the last published contact_level[*] == 0 because
    #         motion-gated samples are masked.
    collector.samples.clear()
    _drive(monitor, collector, _make_js(effort_val=50.0, vel_val=0.3), 20)
    assert collector.samples, "expected at least one contact_level sample"
    last = collector.samples[-1]
    assert all(v == 0.0 for v in last), \
        f"motion frames should publish zeros, got {last[:3]}..."
    print("PASS motion frames masked to zero")

    # Step C: stop motion. After motion_settle_n frames of near-zero velocity,
    #         the lamp should go back to the normalized contact level.
    collector.samples.clear()
    _drive(monitor, collector, _make_js(effort_val=50.0, vel_val=0.0), 20)
    last = collector.samples[-1]
    assert all(v > 0.0 for v in last), \
        f"settled frames with high effort should light, got {last[:3]}..."
    print(f"PASS settled lamps active, levels like {round(last[0], 2)}")

    # Step D: inject a single high-velocity frame -> lamp must drop to 0 again.
    collector.samples.clear()
    _drive(monitor, collector, _make_js(effort_val=50.0, vel_val=0.3), 1)
    last = collector.samples[-1]
    assert all(v == 0.0 for v in last), \
        f"single motion frame should reset stability, got {last[:3]}..."
    print("PASS single motion frame drops lamp")

    monitor.destroy_node()
    collector.destroy_node()
    rclpy.shutdown()
    print("\nALL MOTION GATING TESTS PASSED")


if __name__ == "__main__":
    sys.exit(main() or 0)
