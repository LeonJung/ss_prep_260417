#!/usr/bin/env python3
"""Unit tests for raw_to_joint_deg() and compute_thumb_cmc().

Runnable without manus_ros2_msgs: we import from the module files directly,
bypassing the package's top-level import (retarget_node imports ManusGlove).
"""
import importlib.util
import math
import os
import sys

PKG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "manus_dg5f_retarget"
)


def _load(modname, path):
    spec = importlib.util.spec_from_file_location(modname, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# Load the modules without triggering retarget_node's manus_ros2_msgs import.
thumb_cmc = _load("thumb_cmc", os.path.join(PKG, "thumb_cmc.py"))

# retarget_node imports manus_ros2_msgs; we only need raw_to_joint_deg from it.
# Copy the function over by extracting its source via exec on a stub.
_stub = {}
with open(os.path.join(PKG, "retarget_node.py")) as f:
    src = f.read()
# Strip the manus_ros2_msgs / control_msgs imports + the Node class so we can
# exec just the helpers. The helpers are pure python / math.
helpers = src.split("class ManusDg5fRetarget")[0]
helpers = helpers.replace(
    "from control_msgs.msg import MultiDOFCommand\n", ""
).replace(
    "from manus_ros2_msgs.msg import ManusGlove\n", ""
).replace(
    "from .thumb_cmc import compute_thumb_cmc\n", ""
).replace(
    "import rclpy\n", ""
).replace(
    "from rclpy.node import Node\n", ""
)
exec(helpers, _stub)  # noqa: S102 — trusted source

raw_to_joint_deg = _stub["raw_to_joint_deg"]
RIGHT_JOINT_LIMITS = _stub["RIGHT_JOINT_LIMITS"]


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def test_neutral_input():
    out = raw_to_joint_deg([0.0] * 20)
    # Thumb CMC default (coupled-from-stretch): 58.5 - 0 = 58.5 deg
    assert approx(out[0], 58.5), f"thumb CMC neutral expected 58.5, got {out[0]}"
    # Thumb MCP: q[0]+20 = 0+20 = 20
    assert approx(out[1], 20.0)
    # Index MCPStretch directly forwarded
    assert approx(out[5], 0.0)
    print("PASS test_neutral_input")


def test_index_flex():
    q = [0.0] * 20
    q[4] = 10.0   # IndexSpread
    q[5] = 60.0   # IndexMCPStretch
    q[6] = 70.0   # IndexPIPStretch -> qd[6] = q[6] - 40 = 30
    q[7] = 40.0   # IndexDIPStretch
    out = raw_to_joint_deg(q)
    assert approx(out[4], 10.0)
    assert approx(out[5], 60.0)
    assert approx(out[6], 30.0), f"index PIP offset expected 30, got {out[6]}"
    assert approx(out[7], 40.0)
    print("PASS test_index_flex")


def test_pinky_branch():
    # Trigger the "big" branch: q[17] > 55 and q[18] > 25
    q = [0.0] * 20
    q[16] = -15.0
    q[17] = 60.0
    q[18] = 30.0
    q[19] = 20.0
    out = raw_to_joint_deg(q)
    # q[17]>55 AND q[18]>25 -> qd[16] = abs(-15)*2 = 30
    assert approx(out[16], 30.0)
    print("PASS test_pinky_branch (big)")

    # Small branch
    q[17] = 30.0
    out = raw_to_joint_deg(q)
    assert approx(out[16], 15.0 / 1.5)  # 10
    print("PASS test_pinky_branch (small)")


def test_thumb_cmc_fixed():
    assert thumb_cmc.compute_thumb_cmc("fixed", 10.0, 40.0,
                                       fixed_value_rad=0.3) == 0.3
    print("PASS test_thumb_cmc_fixed")


def test_thumb_cmc_coupled():
    # offset 58.5, gain_stretch 1.0, gain_spread 0.0
    # -> (58.5 - 40.0 + 0.0*10) * DEG
    r = thumb_cmc.compute_thumb_cmc(
        "coupled", 10.0, 40.0, offset_deg=58.5,
        gain_stretch=1.0, gain_spread=0.0
    )
    assert approx(r, 18.5 * math.pi / 180.0, tol=1e-9), f"got {r}"
    print("PASS test_thumb_cmc_coupled")


def test_limits_cover_20_joints():
    assert len(RIGHT_JOINT_LIMITS) == 20
    for lo, hi in RIGHT_JOINT_LIMITS.values():
        assert lo <= hi
    print("PASS test_limits_cover_20_joints")


if __name__ == "__main__":
    test_neutral_input()
    test_index_flex()
    test_pinky_branch()
    test_thumb_cmc_fixed()
    test_thumb_cmc_coupled()
    test_limits_cover_20_joints()
    print("\nALL UNIT TESTS PASSED")
