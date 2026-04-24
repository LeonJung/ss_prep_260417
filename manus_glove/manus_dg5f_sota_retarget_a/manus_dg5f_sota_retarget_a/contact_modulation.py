"""Optional (b)-layer: contact-aware reference modulation.

Off by default. When `contact_aware` is true, the node subscribes to
`/dg5f_<side>/contact_level` (Float32MultiArray, 20 values in [0,1]-ish)
and blends the SLSQP IK output toward a "hold" reference as contact rises,
so the PID doesn't keep pushing once the finger has grabbed an object.

Blend:
    q_out = (1 - alpha) * q_ik + alpha * q_hold
    alpha = clip( (contact - c_lo) / (c_hi - c_lo), 0, 1 ) * max_blend

`q_hold` is the last measured joint state (subscribed from
`/dg5f_<side>/joint_states`). If joint_states is unavailable, falls back
to the previously published reference (i.e., freeze).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ContactConfig:
    enabled: bool = False
    c_lo: float = 0.30
    c_hi: float = 0.80
    max_blend: float = 0.85
    velocity_clamp_rad_s: float = 1.5  # cap per-joint command rate under contact


def modulate(
    q_ik: np.ndarray,
    q_prev_ref: np.ndarray,
    q_meas: Optional[np.ndarray],
    contact: Optional[np.ndarray],
    cfg: ContactConfig,
    dt: float,
) -> np.ndarray:
    if not cfg.enabled or contact is None:
        return q_ik

    alpha = np.clip((contact - cfg.c_lo) / max(cfg.c_hi - cfg.c_lo, 1e-6), 0.0, 1.0)
    alpha = alpha * cfg.max_blend

    q_hold = q_meas if q_meas is not None else q_prev_ref
    q_out = (1.0 - alpha) * q_ik + alpha * q_hold

    # Velocity clamp (per joint) when under contact
    if dt > 0.0:
        max_step = cfg.velocity_clamp_rad_s * dt
        dq = np.clip(q_out - q_prev_ref, -max_step, max_step)
        # blend clamp strength with alpha so free joints are unrestricted
        q_out = q_prev_ref + (1.0 - alpha) * (q_out - q_prev_ref) + alpha * dq

    return q_out
