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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
    def __init__(self, topic: str, expected_side: str, accept_any_side: bool = False):
        super().__init__("calib_wizard")
        self._topic = topic
        self._expected = expected_side.lower()
        self._accept_any = accept_any_side
        self._buf: List[ManusGlove] = []
        self._raw_count = 0    # all msgs received
        self._sides_seen = {}  # what msg.side values we've actually seen

        # Use BEST_EFFORT to match the most common Manus driver QoS. A
        # BEST_EFFORT subscriber is compatible with both RELIABLE and
        # BEST_EFFORT publishers; the reverse (RELIABLE sub + BEST_EFFORT
        # pub) silently drops everything, which is exactly the failure
        # mode we want to avoid.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=50)
        self._cap = self.create_subscription(ManusGlove, topic, self._on_glove, qos)
        self.get_logger().info(
            f"calib_wizard subscribed to {topic} "
            f"(expected side={self._expected!r}, accept_any={accept_any_side}, "
            f"reliability=BEST_EFFORT)")

    def _on_glove(self, msg: ManusGlove):
        self._raw_count += 1
        side = (msg.side or "").lower()
        self._sides_seen[side] = self._sides_seen.get(side, 0) + 1
        if not self._accept_any and side and side != self._expected:
            return
        self._buf.append(msg)
        if len(self._buf) > 200:
            self._buf = self._buf[-200:]

    def diagnostic(self) -> str:
        return (
            f"raw_msgs={self._raw_count}, accepted={len(self._buf)}, "
            f"sides_seen={self._sides_seen}, topic={self._topic}, "
            f"expected_side={self._expected!r}"
        )

    def reset_buffer(self):
        self._buf = []

    def latest_count(self) -> int:
        return len(self._buf)

    def collect(self, n_frames: int, timeout_s: float = 5.0) -> Dict[str, float]:
        """Return averaged ergo dict over the next `n_frames` glove messages."""
        self.reset_buffer()
        raw_at_start = self._raw_count
        t0 = time.time()
        while len(self._buf) < n_frames:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout_s:
                break
        if len(self._buf) < 5:
            raw_seen = self._raw_count - raw_at_start
            raise TimeoutError(
                f"only got {len(self._buf)} accepted glove frames in {timeout_s:.1f} s "
                f"(raw msgs received in window: {raw_seen}). {self.diagnostic()}"
            )
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
    p.add_argument("--accept-any-side", action="store_true",
                   help="bypass the msg.side filter (use if your Manus driver "
                        "publishes side='' or a wrong value)")
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
        node = CaptureNode(f"/manus_glove_{glove_id}", side,
                           accept_any_side=args.accept_any_side)
        # Brief warm-up so the subscription has a chance to receive data.
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node._raw_count == 0:
            print(_c(
                f"WARN: zero /manus_glove_{glove_id} messages seen during 2-s "
                "warmup.", YELLOW))
            print(_c("  Diagnostic checks:", YELLOW))
            print(_c(
                f"    ros2 topic hz /manus_glove_{glove_id}\n"
                f"    ros2 topic info -v /manus_glove_{glove_id}\n"
                f"    ros2 topic echo --once /manus_glove_{glove_id}", DIM))
        elif len(node._buf) == 0 and node._raw_count > 0:
            print(_c(
                f"WARN: {node._raw_count} raw msgs seen but 0 accepted by "
                f"side filter (expected {side!r}).", YELLOW))
            print(_c(f"  Sides observed: {node._sides_seen}", YELLOW))
            print(_c(
                "  Re-run with --accept-any-side to skip the side filter.",
                YELLOW))
        else:
            print(_c(
                f"  glove warmup OK: {node._raw_count} msgs, "
                f"sides_seen={node._sides_seen}", DIM))

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
