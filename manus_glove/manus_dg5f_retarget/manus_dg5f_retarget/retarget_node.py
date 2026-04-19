"""Manus ergonomics -> DG5F right-hand pid_all reference.

Subscribes to /manus_glove_1 (manus_ros2_msgs/ManusGlove), extracts the 20
ergonomics fields for the right hand, applies the empirical per-joint formula
originally found in dg5f_ros2/dg5f_driver/script/manus_retarget/manus_retarget.py
(the values there were trial-calibrated against a real glove+DG5F pair, so we
keep the same transforms here and expose the tunable parts via yaml), and
publishes control_msgs/MultiDOFCommand on
/dg5f_right/rj_dg_pospid/reference.

Thumb CMC (rj_dg_1_1) is the one axis Manus does not measure — it is
supplied by thumb_cmc.py in one of three modes (fixed / coupled /
raw_nodes_ik).
"""

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from control_msgs.msg import MultiDOFCommand
from manus_ros2_msgs.msg import ManusGlove

from .thumb_cmc import compute_thumb_cmc

DEG = math.pi / 180.0

RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

# Manus ergonomics field names, in the same order manus_retarget.py used.
# Index in this list is the q_deg[i] slot referenced by the transforms below.
ERGO_KEYS = [
    "ThumbMCPSpread",  "ThumbMCPStretch", "ThumbPIPStretch", "ThumbDIPStretch",
    "IndexSpread",     "IndexMCPStretch", "IndexPIPStretch", "IndexDIPStretch",
    "MiddleSpread",    "MiddleMCPStretch","MiddlePIPStretch","MiddleDIPStretch",
    "RingSpread",      "RingMCPStretch",  "RingPIPStretch",  "RingDIPStretch",
    "PinkySpread",     "PinkyMCPStretch", "PinkyPIPStretch", "PinkyDIPStretch",
]

# URDF joint limits for the right hand (extracted from dg5f_right.urdf).
# (lower, upper) in radians.
RIGHT_JOINT_LIMITS = {
    "rj_dg_1_1": (-0.3839724354387525, 0.8901179185171081),
    "rj_dg_1_2": (-math.pi, 0.0),
    "rj_dg_1_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_1_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_2_1": (-0.4188790204786391, 0.6108652381980153),
    "rj_dg_2_2": (0.0, 2.007128639793479),
    "rj_dg_2_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_2_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_3_1": (-0.6108652381980153, 0.6108652381980153),
    "rj_dg_3_2": (0.0, 1.9547687622336491),
    "rj_dg_3_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_3_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_4_1": (-0.6108652381980153, 0.4188790204786391),
    "rj_dg_4_2": (0.0, 1.9024088846738192),
    "rj_dg_4_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_4_4": (-math.pi / 2, math.pi / 2),
    "rj_dg_5_1": (-0.017453292519943295, 1.0471975511965976),
    "rj_dg_5_2": (-0.4188790204786391, 0.6108652381980153),
    "rj_dg_5_3": (-math.pi / 2, math.pi / 2),
    "rj_dg_5_4": (-math.pi / 2, math.pi / 2),
}

# Empirically-tuned constants copied from the original manus_retarget.py,
# kept as module defaults. Overridable via yaml parameters.
# Signs (dir_arr) for right hand: qd[i] *= DIR_RIGHT[i]
DIR_RIGHT_DEFAULT = [
     1, -1,  1,  1,
    -1,  1,  1,  1,
    -1,  1,  1,  1,
    -1,  1,  1,  1,
     1, -1,  1,  1,
]
# Calibration multipliers applied after the raw q_deg -> qd transform.
CALIB_DEFAULT = [
    1.0, 1.6, 1.3, 1.3,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.0, 1.0,
]

# Per-joint postprocess: qd[i] >= 0 forced? qd[i] <= 0 forced?
# Values copied from manus_retarget._compute_mqd_from_q right-hand branch.
# "+=0" means "if >0 force 0" (prevents positive / forward bend)
# "-=0" means "if <0 force 0" (prevents negative / back bend)
# Indices in [4, 8, 12, 16, 17] are skipped (pinky base quirks + spreads).
POSTPROC_RIGHT = {
    0:  "skip",       # thumb CMC handled separately
    1:  "no_positive",
    2:  "no_negative", 3:  "no_negative",
    4:  "skip",
    5:  "no_negative", 6:  "no_negative", 7:  "no_negative",
    8:  "skip",
    9:  "no_negative", 10: "no_negative", 11: "no_negative",
    12: "skip",
    13: "no_negative", 14: "no_negative", 15: "no_negative",
    16: "skip",
    17: "skip",
    18: "no_negative", 19: "no_negative",
}


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def raw_to_joint_deg(q_deg: List[float]) -> List[float]:
    """q_deg (20 ergonomics values in deg) -> qd (20 DG5F targets in deg).

    Pre-calibration, pre-direction, pre-clamp. Mirrors the formulas from
    the original manus_retarget.py. Thumb CMC slot (index 0) is computed
    here for the coupled path; callers may overwrite it according to the
    chosen thumb_cmc_mode.
    """
    n = 20
    q = list(q_deg[:n]) + [0.0] * max(0, n - len(q_deg))
    qd = [0.0] * n

    # --- Thumb ---
    qd[0] = 58.5 - q[1]                  # CMC coupled-from-MCPStretch (deg)
    qd[1] = q[0] + 20.0                  # MCP flex
    qd[2] = q[2]                         # PIP
    qd[3] = 0.5 * (q[2] + q[3])          # DIP averaged

    # --- Index / Middle / Ring (2/3/4) ---
    qd[4] = q[4];  qd[5]  = q[5];  qd[6]  = q[6] - 40.0;  qd[7]  = q[7]
    qd[8] = q[8];  qd[9]  = q[9];  qd[10] = q[10] - 30.0; qd[11] = q[11]
    qd[12] = q[12]; qd[13] = q[13]; qd[14] = q[14]; qd[15] = q[15]

    # --- Pinky (5) — conditional branch on flex depth ---
    if q[17] > 55.0 and q[18] > 25.0:
        qd[16] = abs(q[16]) * 2.0
    else:
        qd[16] = abs(q[16]) / 1.5
    qd[17] = q[16]
    qd[18] = q[17]
    qd[19] = q[18]

    return qd


class ManusDg5fRetarget(Node):
    def __init__(self):
        super().__init__("manus_dg5f_retarget")

        self.declare_parameter("input_topic", "/manus_glove_1")
        self.declare_parameter("output_topic", "/dg5f_right/rj_dg_pospid/reference")
        self.declare_parameter("expected_side", "right")  # "right" | "left" | "any"
        self.declare_parameter("thumb_cmc_mode", "fixed")  # fixed | coupled | raw_nodes_ik
        self.declare_parameter("thumb_cmc_fixed_value_rad", 0.0)
        self.declare_parameter("thumb_cmc_offset_deg", 58.5)
        self.declare_parameter("thumb_cmc_gain_stretch", 1.0)
        self.declare_parameter("thumb_cmc_gain_spread", 0.0)
        self.declare_parameter("calib", CALIB_DEFAULT)
        self.declare_parameter("dir_sign", [float(s) for s in DIR_RIGHT_DEFAULT])
        self.declare_parameter("clamp_to_urdf_limits", True)
        self.declare_parameter("apply_postprocess", True)

        self._in_topic = self.get_parameter("input_topic").value
        self._out_topic = self.get_parameter("output_topic").value
        self._expected_side = str(self.get_parameter("expected_side").value).lower()
        self._cmc_mode = str(self.get_parameter("thumb_cmc_mode").value)
        self._cmc_fixed = float(self.get_parameter("thumb_cmc_fixed_value_rad").value)
        self._cmc_offset = float(self.get_parameter("thumb_cmc_offset_deg").value)
        self._cmc_gain_stretch = float(self.get_parameter("thumb_cmc_gain_stretch").value)
        self._cmc_gain_spread = float(self.get_parameter("thumb_cmc_gain_spread").value)
        self._calib = [float(x) for x in self.get_parameter("calib").value]
        self._dir = [float(x) for x in self.get_parameter("dir_sign").value]
        self._clamp_limits = bool(self.get_parameter("clamp_to_urdf_limits").value)
        self._postprocess = bool(self.get_parameter("apply_postprocess").value)

        if len(self._calib) != 20 or len(self._dir) != 20:
            raise ValueError("calib / dir_sign must have exactly 20 entries")

        self._sub = self.create_subscription(
            ManusGlove, self._in_topic, self._on_glove, 10
        )
        self._pub = self.create_publisher(MultiDOFCommand, self._out_topic, 10)

        self.get_logger().info(
            f"manus_dg5f_retarget: {self._in_topic} -> {self._out_topic} "
            f"| thumb_cmc_mode={self._cmc_mode} "
            f"| expected_side={self._expected_side}"
        )

    def _on_glove(self, msg: ManusGlove):
        side = (msg.side or "").lower()
        if self._expected_side != "any" and side != self._expected_side:
            # silently ignore the other hand
            return

        ergo: Dict[str, float] = {}
        for e in msg.ergonomics:
            ergo[e.type] = float(e.value)

        # Strip optional "Left"/"Right" prefix sometimes present in ergo types.
        for prefix in ("Right", "Left"):
            if any(k.startswith(prefix) for k in ergo):
                ergo = {
                    (k[len(prefix):] if k.startswith(prefix) else k): v
                    for k, v in ergo.items()
                }
                break

        q_deg = [ergo.get(k, 0.0) for k in ERGO_KEYS]
        qd_deg = raw_to_joint_deg(q_deg)

        # Thumb CMC override by mode.
        cmc_rad = compute_thumb_cmc(
            mode=self._cmc_mode,
            mcp_spread_deg=q_deg[0],
            mcp_stretch_deg=q_deg[1],
            raw_nodes=getattr(msg, "raw_nodes", None),
            fixed_value_rad=self._cmc_fixed,
            offset_deg=self._cmc_offset,
            gain_stretch=self._cmc_gain_stretch,
            gain_spread=self._cmc_gain_spread,
        )

        # deg -> rad, apply calib and sign.
        qd_rad = [(qd_deg[i] * DEG) * self._calib[i] * self._dir[i]
                  for i in range(20)]

        # Thumb CMC replacement: `cmc_rad` already applied sign in coupled mode
        # via our offset math; for `fixed` the caller sets value directly.
        qd_rad[0] = cmc_rad

        # Postprocess (prevent backward/forward bend) — skip slot 0 (CMC).
        if self._postprocess:
            for i, rule in POSTPROC_RIGHT.items():
                if rule == "skip":
                    continue
                if rule == "no_positive" and qd_rad[i] > 0.0:
                    qd_rad[i] = 0.0
                elif rule == "no_negative" and qd_rad[i] < 0.0:
                    qd_rad[i] = 0.0

        # Clamp to URDF limits.
        if self._clamp_limits:
            for i, name in enumerate(RIGHT_JOINT_NAMES):
                lo, hi = RIGHT_JOINT_LIMITS[name]
                qd_rad[i] = _clamp(qd_rad[i], lo, hi)

        out = MultiDOFCommand()
        out.dof_names = list(RIGHT_JOINT_NAMES)
        out.values = qd_rad
        out.values_dot = [0.0] * 20
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ManusDg5fRetarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
