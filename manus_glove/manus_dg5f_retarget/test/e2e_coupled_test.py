#!/usr/bin/env python3
"""E2E: thumb CMC coupled-mode + joint limit clamping.

With thumb_cmc_mode=coupled and offset/gain defaults (58.5 / 1.0 / 0.0):
  - MCPStretch=0 -> cmc_rad = 58.5 deg = 1.021 rad  (clamp hi=0.89 -> 0.89)
  - MCPStretch=58.5 -> cmc_rad = 0.0 rad
  - MCPStretch=90 -> cmc_rad = -31.5 deg = -0.55 rad (clamp lo=-0.38 -> -0.38)

This also exercises the URDF limit clamping (rj_dg_1_1 range -0.384..0.890).
"""
import os
import subprocess
import sys
import time

try:
    from manus_glove_sim.msg_builder import build_manus_glove_msg
except ImportError as e:
    print(f"SKIP: {e}")
    sys.exit(0)

import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove
from control_msgs.msg import MultiDOFCommand


class Harness(Node):
    def __init__(self):
        super().__init__("e2e_coupled_harness")
        self.samples = []
        self.create_subscription(
            MultiDOFCommand, "/dg5f_right/rj_dg_pospid/reference",
            lambda m: self.samples.append(list(m.values)), 10,
        )
        self.pub = self.create_publisher(ManusGlove, "/manus_glove_1", 10)

    def publish(self, vals):
        self.pub.publish(build_manus_glove_msg(vals, side="Right", glove_id=1))


def _run():
    env = os.environ.copy()
    retarget = subprocess.Popen(
        ["ros2", "run", "manus_dg5f_retarget", "retarget_node",
         "--ros-args", "-p", "thumb_cmc_mode:=coupled",
         "-p", "thumb_cmc_offset_deg:=58.5",
         "-p", "thumb_cmc_gain_stretch:=1.0",
         "-p", "thumb_cmc_gain_spread:=0.0"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        rclpy.init()
        node = Harness()

        # Phase 1: MCPStretch=0  -> expected ~1.02 rad clamped to 0.89
        for _ in range(30):
            node.publish({"ThumbMCPStretch": 0.0})
            rclpy.spin_once(node, timeout_sec=0.05)
        p1 = [s[0] for s in node.samples[-20:]]

        node.samples.clear()

        # Phase 2: MCPStretch=58.5 -> ~0.0 rad
        for _ in range(30):
            node.publish({"ThumbMCPStretch": 58.5})
            rclpy.spin_once(node, timeout_sec=0.05)
        p2 = [s[0] for s in node.samples[-20:]]

        node.samples.clear()

        # Phase 3: MCPStretch=90 -> expected -0.55 rad clamped to -0.38
        for _ in range(30):
            node.publish({"ThumbMCPStretch": 90.0})
            rclpy.spin_once(node, timeout_sec=0.05)
        p3 = [s[0] for s in node.samples[-20:]]

        if not (p1 and p2 and p3):
            print("FAIL: empty phase samples")
            return 2

        avg = lambda xs: sum(xs) / len(xs)  # noqa: E731
        v1, v2, v3 = avg(p1), avg(p2), avg(p3)
        print(f"phase1 (stretch=0)    rj_dg_1_1 avg = {v1:+.3f} rad  (expected ~ 0.89 clamped)")
        print(f"phase2 (stretch=58.5) rj_dg_1_1 avg = {v2:+.3f} rad  (expected ~ 0.00)")
        print(f"phase3 (stretch=90)   rj_dg_1_1 avg = {v3:+.3f} rad  (expected ~ -0.38 clamped)")

        fails = []
        if not (0.85 <= v1 <= 0.91):
            fails.append(f"phase1 clamp mismatch: {v1:.3f} not in [0.85, 0.91]")
        if not (-0.05 <= v2 <= 0.05):
            fails.append(f"phase2 zero mismatch: {v2:.3f} not in [-0.05, 0.05]")
        if not (-0.40 <= v3 <= -0.36):
            fails.append(f"phase3 clamp mismatch: {v3:.3f} not in [-0.40, -0.36]")

        if fails:
            for f in fails:
                print("FAIL:", f)
            return 3

        # Also verify monotonic decrease v1 > v2 > v3
        if not (v1 > v2 > v3):
            print(f"FAIL: not monotonic  v1={v1:.3f} v2={v2:.3f} v3={v3:.3f}")
            return 4

        print("OK: coupled mode & URDF clamping behave as expected")
        node.destroy_node()
        rclpy.shutdown()
        return 0
    finally:
        import signal as _sig
        try:
            os.killpg(os.getpgid(retarget.pid), _sig.SIGKILL)
        except Exception:
            pass
        try:
            retarget.wait(timeout=2)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(_run())
