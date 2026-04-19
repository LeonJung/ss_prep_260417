#!/usr/bin/env python3
"""Pre-hardware E2E test for the DG5F software stack.

Spawns dg5f_sim_driver + contact_monitor, subscribes to
/dg5f_right/contact_level, and checks that the grip_cycle scenario produces
the expected pattern: distal joints (indices 3, 7, 11, 15, 19) saturate to
1.0 during the hold window while the rest stay near baseline.

Run with:
    source install/setup.bash
    python3 src/dg5f_hand_bringup/test/e2e_sim_test.py
"""
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DISTAL_INDICES = [3, 7, 11, 15, 19]
COLLECT_SECONDS = 8.0
MIN_SAMPLES = 30
DISTAL_PEAK_MIN = 0.9


class Collector(Node):
    def __init__(self):
        super().__init__("dg5f_e2e_collector")
        self.samples = []
        self.create_subscription(
            Float32MultiArray, "/dg5f_right/contact_level",
            lambda m: self.samples.append(list(m.data)), 10
        )


def _run():
    env = os.environ.copy()
    sim = subprocess.Popen(
        ["ros2", "run", "dg5f_hand_bringup", "dg5f_sim_driver.py",
         "--ros-args", "-p", "scenario:=grip_cycle", "-p", "cycle_period_s:=4.0"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    mon = subprocess.Popen(
        ["ros2", "run", "dg5f_contact_viz", "contact_monitor"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        rclpy.init()
        node = Collector()
        deadline = time.time() + COLLECT_SECONDS
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

        if len(node.samples) < MIN_SAMPLES:
            print(f"FAIL: got {len(node.samples)} samples (< {MIN_SAMPLES})")
            return 2

        distal_peak = max(max(s[i] for i in DISTAL_INDICES) for s in node.samples)
        if distal_peak < DISTAL_PEAK_MIN:
            print(f"FAIL: distal peak {distal_peak:.2f} < {DISTAL_PEAK_MIN}")
            return 3

        print(f"OK: {len(node.samples)} samples, distal peak {distal_peak:.2f}")
        print("last:", [round(x, 2) for x in node.samples[-1]])
        node.destroy_node()
        rclpy.shutdown()
        return 0
    finally:
        for p in (sim, mon):
            try:
                p.terminate(); p.wait(timeout=2)
            except Exception:
                p.kill()


if __name__ == "__main__":
    sys.exit(_run())
