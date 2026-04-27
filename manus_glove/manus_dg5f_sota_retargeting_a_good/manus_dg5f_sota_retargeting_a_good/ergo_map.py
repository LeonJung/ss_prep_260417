"""Ergonomics -> DG5F per-joint angles (ergo-mapped warm start).

Mirrors raw_to_joint_deg / POSTPROC / calib logic from the original
`manus_dg5f_retarget.retarget_node`. Reused here as the optimizer's prior /
warm start (q0). This module is side-agnostic — per-hand sign + limits are
applied by the caller via `dir_sign` and URDF-derived bounds.
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

DEG = math.pi / 180.0

ERGO_KEYS = [
    "ThumbMCPSpread",  "ThumbMCPStretch", "ThumbPIPStretch", "ThumbDIPStretch",
    "IndexSpread",     "IndexMCPStretch", "IndexPIPStretch", "IndexDIPStretch",
    "MiddleSpread",    "MiddleMCPStretch","MiddlePIPStretch","MiddleDIPStretch",
    "RingSpread",      "RingMCPStretch",  "RingPIPStretch",  "RingDIPStretch",
    "PinkySpread",     "PinkyMCPStretch", "PinkyPIPStretch", "PinkyDIPStretch",
]

CALIB_DEFAULT = [
    1.0, 1.6, 1.3, 1.3,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.3, 1.7,
    1.0, 1.0, 1.0, 1.0,
]

DIR_RIGHT = np.array([
     1, -1,  1,  1,
    -1,  1,  1,  1,
    -1,  1,  1,  1,
    -1,  1,  1,  1,
     1, -1,  1,  1,
], dtype=float)

DIR_LEFT = np.array([
    -1,  1, -1, -1,
     1,  1,  1,  1,
     1,  1,  1,  1,
     1,  1,  1,  1,
    -1,  1,  1,  1,
], dtype=float)

POSTPROC_RIGHT = {
    0: "skip", 1: "no_positive",
    2: "no_negative", 3: "no_negative",
    4: "skip",
    5: "no_negative", 6: "no_negative", 7: "no_negative",
    8: "skip",
    9: "no_negative", 10: "no_negative", 11: "no_negative",
    12: "skip",
    13: "no_negative", 14: "no_negative", 15: "no_negative",
    16: "skip", 17: "skip",
    18: "no_negative", 19: "no_negative",
}
POSTPROC_LEFT = {
    0: "skip", 1: "no_negative",
    2: "no_positive", 3: "no_positive",
    4: "skip",
    5: "no_negative", 6: "no_negative", 7: "no_negative",
    8: "skip",
    9: "no_negative", 10: "no_negative", 11: "no_negative",
    12: "skip",
    13: "no_negative", 14: "no_negative", 15: "no_negative",
    16: "skip", 17: "skip",
    18: "no_negative", 19: "no_negative",
}


def raw_to_joint_deg(q_deg: List[float]) -> List[float]:
    """Symmetric ergonomics -> DG5F joint angles in deg."""
    n = 20
    q = list(q_deg[:n]) + [0.0] * max(0, n - len(q_deg))
    qd = [0.0] * n
    # Thumb
    qd[0] = 58.5 - q[1]
    qd[1] = q[0] + 20.0
    qd[2] = q[2]
    qd[3] = 0.5 * (q[2] + q[3])
    # Index / Middle / Ring
    qd[4] = q[4];  qd[5]  = q[5];  qd[6]  = q[6] - 40.0;  qd[7]  = q[7]
    qd[8] = q[8];  qd[9]  = q[9];  qd[10] = q[10] - 30.0; qd[11] = q[11]
    qd[12] = q[12]; qd[13] = q[13]; qd[14] = q[14]; qd[15] = q[15]
    # Pinky
    if q[17] > 55.0 and q[18] > 25.0:
        qd[16] = abs(q[16]) * 2.0
    else:
        qd[16] = abs(q[16]) / 1.5
    qd[17] = q[16]
    qd[18] = q[17]
    qd[19] = q[18]
    return qd


def ergo_to_q0_rad(
    ergo: Dict[str, float],
    calib: np.ndarray,
    dir_sign: np.ndarray,
    postproc: Dict[int, str],
    thumb_cmc_rad: float,
) -> np.ndarray:
    """Apply raw_to_joint_deg + calib + dir + postprocess. Returns q0 in rad.

    Mirrors manus_dg5f_retarget.retarget_node but without the CMC computation
    (passed in) and without URDF clamping (that becomes the SLSQP bound).
    """
    q_deg = [float(ergo.get(k, 0.0)) for k in ERGO_KEYS]
    qd_deg = raw_to_joint_deg(q_deg)
    q0 = np.zeros(20)
    for i in range(20):
        q0[i] = qd_deg[i] * DEG * calib[i] * dir_sign[i]
    # Thumb CMC slot gets its own pipeline; apply side sign.
    q0[0] = thumb_cmc_rad * dir_sign[0]
    for i, rule in postproc.items():
        if rule == "skip":
            continue
        if rule == "no_positive" and q0[i] > 0.0:
            q0[i] = 0.0
        elif rule == "no_negative" and q0[i] < 0.0:
            q0[i] = 0.0
    return q0


def compute_thumb_cmc(mode: str, spread_deg: float, stretch_deg: float,
                      fixed_value_rad: float = 0.0,
                      offset_deg: float = 58.5,
                      gain_stretch: float = 1.0,
                      gain_spread: float = 0.0) -> float:
    """Resolve thumb CMC in radians.

    Mirrors manus_dg5f_retarget.thumb_cmc_modes:
      - "fixed"   : constant `fixed_value_rad`. Matches original's default.
      - "coupled" : offset - gain_stretch*stretch + gain_spread*spread (deg).
    Callers apply the per-hand sign on the result.
    """
    if mode == "coupled":
        val_deg = offset_deg - gain_stretch * stretch_deg + gain_spread * spread_deg
        return val_deg * DEG
    return float(fixed_value_rad)


# Back-compat shim: the earlier implementation only had the linear formula.
def compute_thumb_cmc_fixed(spread_deg: float, stretch_deg: float,
                            offset_deg: float = 58.5,
                            gain_stretch: float = 1.0,
                            gain_spread: float = 0.0) -> float:
    return compute_thumb_cmc("coupled", spread_deg, stretch_deg,
                             offset_deg=offset_deg,
                             gain_stretch=gain_stretch,
                             gain_spread=gain_spread)
