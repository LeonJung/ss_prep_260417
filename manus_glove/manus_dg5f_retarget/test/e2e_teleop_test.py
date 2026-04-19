#!/usr/bin/env python3
"""Pre-hardware Manus->DG5F teleop regression.

Publishes a hand-crafted manus_ros2_msgs/ManusGlove on /manus_glove_1 and
verifies that manus_dg5f_retarget republishes a 20-DOF MultiDOFCommand on
/dg5f_right/rj_dg_pospid/reference whose values are consistent with the
input (thumb CMC fixed to 0, IndexMCPStretch drives rj_dg_2_2, etc.).

Skips gracefully if manus_ros2_msgs is not installed.
"""
import os
import subprocess
import sys
import time

try:
    from manus_glove_sim.msg_builder import build_manus_glove_msg, ERGO_KEYS
except ImportError as e:
    print(f"SKIP: manus_ros2_msgs / manus_glove_sim not available ({e})")
    sys.exit(0)

import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove
from control_msgs.msg import MultiDOFCommand


COLLECT_SECONDS = 6.0
MIN_SAMPLES = 10


class Harness(Node):
    def __init__(self):
        super().__init__("manus_teleop_e2e_harness")
        self.samples = []
        self.create_subscription(
            MultiDOFCommand, "/dg5f_right/rj_dg_pospid/reference",
            lambda m: self.samples.append(list(m.values)), 10,
        )
        self.pub = self.create_publisher(ManusGlove, "/manus_glove_1", 10)

    def publish_pose(self, values_deg):
        msg = build_manus_glove_msg(values_deg, side="Right", glove_id=1)
        self.pub.publish(msg)


def _run():
    env = os.environ.copy()
    retarget = subprocess.Popen(
        ["ros2", "run", "manus_dg5f_retarget", "retarget_node",
         "--ros-args", "-p", "thumb_cmc_mode:=fixed",
         "-p", "thumb_cmc_fixed_value_rad:=0.0"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,  # put in its own process group so we can killpg
    )
    try:
        rclpy.init()
        node = Harness()

        # Step 1: neutral pose
        deadline = time.time() + COLLECT_SECONDS
        tick = 0
        while time.time() < deadline and rclpy.ok():
            tick += 1
            pose = {}
            if tick % 60 > 30:
                # alternate between neutral and an index-only flex
                pose = {"IndexMCPStretch": 60.0, "IndexPIPStretch": 70.0,
                        "IndexDIPStretch": 40.0}
            node.publish_pose(pose)
            rclpy.spin_once(node, timeout_sec=0.05)

        if len(node.samples) < MIN_SAMPLES:
            print(f"FAIL: got {len(node.samples)} samples (< {MIN_SAMPLES})")
            return 2

        n_joints = len(node.samples[-1])
        if n_joints != 20:
            print(f"FAIL: output has {n_joints} joints (expected 20)")
            return 3

        # rj_dg_1_1 must stay at ~0 under fixed thumb_cmc_mode
        max_thumb_cmc = max(abs(s[0]) for s in node.samples)
        if max_thumb_cmc > 0.05:
            print(f"FAIL: thumb CMC (idx 0) drifted to {max_thumb_cmc:.3f} in fixed mode")
            return 4

        # rj_dg_2_2 should have swept non-zero when we set IndexMCPStretch
        max_index_mcp = max(abs(s[5]) for s in node.samples)
        if max_index_mcp < 0.05:
            print(f"FAIL: rj_dg_2_2 never moved (max abs = {max_index_mcp:.3f})")
            return 5

        print(f"OK: {len(node.samples)} samples, 20 joints each, "
              f"thumb_cmc max={max_thumb_cmc:.3f}, rj_dg_2_2 max={max_index_mcp:.3f}")
        print("last:", [round(x, 2) for x in node.samples[-1]])
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
