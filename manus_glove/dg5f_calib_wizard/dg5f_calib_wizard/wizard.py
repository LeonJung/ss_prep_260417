"""dg5f_calib_wizard CLI.

Walks the user through a fixed set of canonical hand poses, captures the
averaged Manus glove ergonomics for each, computes new
manus_dg5f_sota_retarget_a yaml params (calib[20] + pinch sigmoid bounds),
and writes the result to a calibrated yaml file. Repeats up to N
iterations so the user can hot-reload the retarget node and re-run the
wizard with the just-written yaml as the new baseline.

Usage:
    ros2 run dg5f_calib_wizard calib_wizard \\
        --side right \\
        --input-yaml ~/hand_ws/src/manus_glove/manus_dg5f_sota_retarget_a/config/right_hand.yaml \\
        --output-yaml ~/hand_ws/right_hand_calibrated.yaml \\
        --iterations 3 \\
        --glove-id 1
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from manus_ros2_msgs.msg import ManusGlove

from .poses import POSES, Pose
from .tuner import tune
from .yaml_io import load_params, save_params


# ------ ANSI helpers (terminal colours) ------
RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[36m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
DIM = "\033[2m"


def _c(s: str, *codes: str) -> str:
    return "".join(codes) + s + RESET


# ------ ROS2 capture node ------
class CaptureNode(Node):
    def __init__(self, topic: str, expected_side: str):
        super().__init__("calib_wizard")
        self._expected = expected_side.lower()
        self._buf: List[ManusGlove] = []
        self._cap = self.create_subscription(ManusGlove, topic, self._on_glove, 50)
        self.get_logger().info(
            f"calib_wizard subscribed to {topic} (expected side={self._expected})")

    def _on_glove(self, msg: ManusGlove):
        side = (msg.side or "").lower()
        if side and side != self._expected:
            return
        self._buf.append(msg)
        if len(self._buf) > 200:
            self._buf = self._buf[-200:]

    def reset_buffer(self):
        self._buf = []

    def latest_count(self) -> int:
        return len(self._buf)

    def collect(self, n_frames: int, timeout_s: float = 5.0) -> Dict[str, float]:
        """Return averaged ergo dict over the next `n_frames` glove messages."""
        self.reset_buffer()
        t0 = time.time()
        while len(self._buf) < n_frames:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout_s:
                break
        if len(self._buf) < 5:
            raise TimeoutError(
                f"only got {len(self._buf)} glove frames in {timeout_s:.1f} s")
        # Average ergonomics across captured msgs.
        keys: set = set()
        for m in self._buf:
            for e in m.ergonomics:
                keys.add(e.type)
        avg = {k: 0.0 for k in keys}
        cnt = 0
        for m in self._buf[-n_frames:]:
            for e in m.ergonomics:
                avg[e.type] += float(e.value)
            cnt += 1
        if cnt == 0:
            raise RuntimeError("no glove frames after reset (impossible)")
        for k in avg:
            avg[k] /= cnt
        return avg


# ------ wizard helpers ------
def _press_enter(msg: str = "Press ENTER when ready"):
    try:
        input(_c(f"  > {msg}", DIM))
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(1)


def _capture_pose(node: CaptureNode, pose: Pose, n_frames: int = 30) -> Dict[str, float]:
    print()
    print(_c(f"== {pose.name.upper()} ==", BOLD, CYAN))
    print(_c(pose.instruction, CYAN))
    _press_enter("Hold the pose, then press ENTER")
    print(_c("  capturing...", DIM), end="", flush=True)
    avg = node.collect(n_frames=n_frames, timeout_s=8.0)
    print(_c(f" {len(node._buf)} frames OK", GREEN))
    return avg


def _default_input_yaml(side: str) -> str:
    pkg_share = get_package_share_directory("manus_dg5f_sota_retarget_a")
    return os.path.join(pkg_share, "config", f"{side}_hand.yaml")


def _default_urdf_path(side: str) -> str:
    pkg_share = get_package_share_directory("dg5f_description")
    return os.path.join(pkg_share, "urdf", f"dg5f_{side}.urdf")


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="calib_wizard",
                                description="DG5F retargeting calibration wizard")
    p.add_argument("--side", choices=["right", "left"], default="right")
    p.add_argument("--glove-id", type=int, default=None,
                   help="Manus glove id (default: 1 for right, 0 for left)")
    p.add_argument("--input-yaml", default=None,
                   help="baseline yaml (default: manus_dg5f_sota_retarget_a/config/<side>_hand.yaml)")
    p.add_argument("--output-yaml", default=None,
                   help="where to write calibrated yaml (default: ~/hand_ws/<side>_hand_calibrated.yaml)")
    p.add_argument("--iterations", type=int, default=3, help="capture/tune cycles")
    p.add_argument("--frames", type=int, default=30, help="frames to average per pose")
    p.add_argument("--tip-target-mm", type=float, default=1.0,
                   help="tip-to-tip pinch closure target (mm)")
    p.add_argument("--pad-target-mm", type=float, default=1.0,
                   help="pad-to-pad pinch closure target (mm)")
    p.add_argument("--urdf-path", default=None,
                   help="override URDF path (default: dg5f_description share)")
    args = p.parse_args(argv)

    side = args.side
    glove_id = args.glove_id if args.glove_id is not None else (1 if side == "right" else 0)
    in_yaml = args.input_yaml or _default_input_yaml(side)
    out_yaml = args.output_yaml or os.path.expanduser(
        f"~/hand_ws/{side}_hand_calibrated.yaml")
    urdf_path = args.urdf_path or _default_urdf_path(side)

    if not os.path.exists(in_yaml):
        print(_c(f"input yaml not found: {in_yaml}", RED), file=sys.stderr)
        return 2
    if not os.path.exists(urdf_path):
        print(_c(f"URDF not found: {urdf_path}", RED), file=sys.stderr)
        return 2

    print(_c("dg5f_calib_wizard (lite)", BOLD))
    print(f"  side         : {side}")
    print(f"  glove id     : {glove_id} (topic /manus_glove_{glove_id})")
    print(f"  input yaml   : {in_yaml}")
    print(f"  output yaml  : {out_yaml}")
    print(f"  urdf         : {urdf_path}")
    print(f"  iterations   : {args.iterations}")
    print(f"  tip target   : {args.tip_target_mm:.1f} mm")
    print(f"  pad target   : {args.pad_target_mm:.1f} mm")
    print()
    print(_c("This will OVERWRITE the output yaml each iteration.", YELLOW))
    print(_c(
        "Make sure manus_data_publisher (or sim_glove) is running and "
        f"publishing to /manus_glove_{glove_id} with side='{side.capitalize()}'.",
        YELLOW))
    _press_enter("Ready to start?")

    rclpy.init()
    try:
        node = CaptureNode(f"/manus_glove_{glove_id}", side)
        # Brief warm-up so the subscription has a chance to receive data.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_count() == 0:
            print(_c(
                f"WARN: no /manus_glove_{glove_id} messages seen yet. "
                "Check that the driver is running and side matches.", YELLOW))

        cur_yaml_path = in_yaml
        for it in range(1, args.iterations + 1):
            print()
            print(_c(f"### Iteration {it} / {args.iterations}", BOLD, GREEN))
            cfg = load_params(cur_yaml_path)

            captures: Dict[str, Dict[str, float]] = {}
            for pose in POSES:
                captures[pose.name] = _capture_pose(node, pose, n_frames=args.frames)

            print()
            print(_c("Tuning…", BOLD))
            new_cfg = tune(cfg, captures, urdf_path=urdf_path, side=side,
                           tip_target_m=args.tip_target_mm * 1e-3,
                           pad_target_m=args.pad_target_mm * 1e-3,
                           verbose=True)
            save_params(out_yaml, new_cfg, src_path=cur_yaml_path)
            print(_c(f"  wrote {out_yaml}", GREEN))

            # diff summary
            old_calib = np.asarray(cfg["calib"], dtype=float)
            new_calib = np.asarray(new_cfg["calib"], dtype=float)
            diff = new_calib - old_calib
            top = np.argsort(-np.abs(diff))[:5]
            print(_c("  largest calib changes (joint slot, before -> after):", DIM))
            for k in top:
                if abs(diff[k]) > 1e-4:
                    print(_c(
                        f"    slot {k:2d}: {old_calib[k]:.3f} -> {new_calib[k]:.3f} "
                        f"(Δ {diff[k]:+.3f})", DIM))

            if it < args.iterations:
                print()
                print(_c(
                    "Reload your retarget node with --params-file pointing at "
                    f"{out_yaml} (or restart its launch with the new yaml), "
                    "test the pinch quality, then come back here to run "
                    "the next iteration.", YELLOW))
                _press_enter("Press ENTER for the next iteration "
                             "(Ctrl-C to stop)")
                cur_yaml_path = out_yaml

        print()
        print(_c("Done. Final calibrated yaml: ", BOLD) + out_yaml)
        print(_c(
            "To use it, replace the retarget node's --params-file with this "
            "path or copy it into the package's config/ folder and rebuild.",
            DIM))
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
