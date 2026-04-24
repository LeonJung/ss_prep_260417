"""SLSQP IK solver with the "A"-base retargeting objective.

Objective terms (weights loaded from yaml):
  w_prior  * || q - q0 ||^2                      (ergo-mapped warm start)
  w_vel    * || q - q_prev ||^2                  (time smoothness)
  w_orient * Σ_finger  || dir(q) - dir(q0) ||^2  (fingertip direction)
  w_pinch  * Σ_f  g(d_ref_f) * (d_robot_f - d_target_f)^2

Pinch pairs: thumb vs {index, middle, ring, pinky}.
  - d_ref_f    = |tip_thumb(q0) - tip_f(q0)|  (intent from ergo mapping's FK)
  - d_robot_f  = |tip_thumb(q)  - tip_f(q) |
  - d_target_f = rescale(d_ref_f) -> drives closer when pinching
  - g(d)       = sigmoid-based weight (paper's continuous switching)

Warm start q0 = ergo_to_q0_rad(ergo). Bounds = URDF joint limits.
Solver = scipy.optimize.minimize(method="SLSQP") with analytical gradient.
TODO(osqp): reformulate as convex QP in joint-differential space (AC base).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize

from .urdf_fk import FingerChain, fk_finger, jacobian_tip_pos, jacobian_tip_dir


FINGER_SLOT_SLICES = [slice(i * 4, (i + 1) * 4) for i in range(5)]


@dataclass
class AObjectiveWeights:
    prior: float = 1.0
    velocity: float = 0.2
    orient: float = 0.3
    pinch: float = 4.0
    # pinch shape
    pinch_close_ref_m: float = 0.030   # human d_ref below this => strong pinch intent
    pinch_far_ref_m:   float = 0.080   # above this => no pinch
    pinch_target_min_m: float = 0.003  # closed pinch gap on robot
    pinch_rescale_k:   float = 0.6     # target = min + k * (ref - min), clamped
    # Distance residuals are in metres; without scaling, (0.02 m)^2 = 4e-4 is
    # dwarfed by a rad^2 prior term. Work in centimetres internally so pinch
    # weights stay O(1) like the paper's reported 200-400 range (w=4 here
    # behaves like w=4e4 in raw m^2 terms).
    pinch_distance_scale: float = 100.0


@dataclass
class AObjectiveConfig:
    weights: AObjectiveWeights = field(default_factory=AObjectiveWeights)
    finger_chains: List[FingerChain] = field(default_factory=list)  # len = 5
    bounds_lo: np.ndarray = field(default_factory=lambda: np.zeros(20))
    bounds_hi: np.ndarray = field(default_factory=lambda: np.zeros(20))
    max_iter: int = 25
    ftol: float = 1e-5


def _sigmoid_intent(d_ref: float, close: float, far: float) -> float:
    """1.0 when d_ref <= close, 0.0 when d_ref >= far, smooth in between."""
    if d_ref <= close:
        return 1.0
    if d_ref >= far:
        return 0.0
    x = (d_ref - close) / (far - close)
    # cosine ramp, C^1 continuous
    return 0.5 * (1.0 + np.cos(np.pi * x))


def _pinch_target(d_ref: float, cfg: AObjectiveWeights) -> float:
    """Rescaled target pinch distance. Closes gap when d_ref is small."""
    if d_ref <= cfg.pinch_close_ref_m:
        return cfg.pinch_target_min_m
    return cfg.pinch_target_min_m + cfg.pinch_rescale_k * (d_ref - cfg.pinch_close_ref_m)


def _fk_all(chains: List[FingerChain], q: np.ndarray):
    """FK for all 5 fingers; returns list of FK dicts + tip positions in
    palm frame (including each finger's base offset)."""
    fks = []
    tips = np.zeros((5, 3))
    tip_dirs = np.zeros((5, 3))
    for i, ch in enumerate(chains):
        fk = fk_finger(ch, q[FINGER_SLOT_SLICES[i]])
        # fk["tip_pos"] is already in palm frame because chain.origins[0] is
        # already the finger-1 offset from palm. So no extra base translation.
        fks.append(fk)
        tips[i] = fk["tip_pos"]
        tip_dirs[i] = fk["tip_dir"]
    return fks, tips, tip_dirs


def evaluate(cfg: AObjectiveConfig, q: np.ndarray, q0: np.ndarray,
             q_prev: np.ndarray, tips_ref: np.ndarray, dirs_ref: np.ndarray,
             d_refs: np.ndarray):
    """Compute A objective + analytical gradient at q.

    tips_ref / dirs_ref / d_refs are precomputed once per glove frame from q0.
    """
    w = cfg.weights
    chains = cfg.finger_chains

    fks, tips, dirs = _fk_all(chains, q)

    # Scalar terms
    r_prior = q - q0
    r_vel = q - q_prev
    j_prior = w.prior * float(np.dot(r_prior, r_prior))
    j_vel = w.velocity * float(np.dot(r_vel, r_vel))

    # Orient per finger: sum 1 - (dirs[i]·dirs_ref[i])  (equiv to 0.5*||dirs - dirs_ref||^2 on unit vectors)
    j_orient = 0.0
    for i in range(5):
        j_orient += 1.0 - float(np.dot(dirs[i], dirs_ref[i]))
    j_orient *= w.orient

    # Pinch thumb vs finger i=1..4 (index, middle, ring, pinky)
    j_pinch = 0.0
    pinch_info = []
    s = w.pinch_distance_scale
    for fi in range(1, 5):
        v = tips[0] - tips[fi]
        d = float(np.linalg.norm(v)) + 1e-9
        d_ref = float(d_refs[fi])
        tgt = _pinch_target(d_ref, w)
        g = _sigmoid_intent(d_ref, w.pinch_close_ref_m, w.pinch_far_ref_m)
        err_s = (d - tgt) * s
        j_pinch += g * err_s * err_s
        pinch_info.append((fi, d, tgt, g, v, err_s))
    j_pinch *= w.pinch

    total = j_prior + j_vel + j_orient + j_pinch

    # --- gradient ---
    grad = np.zeros(20)
    grad += 2.0 * w.prior * r_prior
    grad += 2.0 * w.velocity * r_vel

    # orient grad
    for i in range(5):
        sl = FINGER_SLOT_SLICES[i]
        Jd = jacobian_tip_dir(chains[i], q[sl], fks[i])
        # d(1 - dirs[i]·dirs_ref[i]) / d q_finger_i = -Jd.T @ dirs_ref[i]
        grad[sl] += -w.orient * (Jd.T @ dirs_ref[i])

    # pinch grad: obj = w_pinch * g * (s * (d - tgt))^2
    #             d obj / dq = 2 * w_pinch * g * s * (s * (d - tgt)) * d d / d q
    #                        = 2 * w_pinch * g * s * err_s * d d / d q
    for (fi, d, tgt, g, v, err_s) in pinch_info:
        Jp_thumb = jacobian_tip_pos(chains[0], q[FINGER_SLOT_SLICES[0]], fks[0])
        Jp_f = jacobian_tip_pos(chains[fi], q[FINGER_SLOT_SLICES[fi]], fks[fi])
        vn = v / d
        d_err_d_qthumb = vn @ Jp_thumb
        d_err_d_qfi = -vn @ Jp_f
        coef = 2.0 * w.pinch * g * s * err_s
        grad[FINGER_SLOT_SLICES[0]] += coef * d_err_d_qthumb
        grad[FINGER_SLOT_SLICES[fi]] += coef * d_err_d_qfi

    return total, grad, {"j_prior": j_prior, "j_vel": j_vel,
                        "j_orient": j_orient, "j_pinch": j_pinch,
                        "tips": tips, "dirs": dirs}


def solve_ik(cfg: AObjectiveConfig, ergo_q0: np.ndarray,
             q_prev: Optional[np.ndarray] = None) -> Tuple[np.ndarray, dict]:
    """Run SLSQP from warm-start q0.

    ergo_q0: (20,) ergo-mapped joint angles (warm start + prior reference).
    q_prev:  (20,) last commanded angles (velocity reg); defaults to q0 if None.
    Returns (q_opt, info_dict).
    """
    q0 = np.clip(ergo_q0, cfg.bounds_lo, cfg.bounds_hi)
    qp = q0 if q_prev is None else np.clip(q_prev, cfg.bounds_lo, cfg.bounds_hi)

    # Precompute reference FK for prior's target direction + d_refs
    _, tips_ref, dirs_ref = _fk_all(cfg.finger_chains, q0)
    d_refs = np.zeros(5)
    for fi in range(1, 5):
        d_refs[fi] = float(np.linalg.norm(tips_ref[0] - tips_ref[fi]))

    def fun_and_grad(q):
        j, g, _info = evaluate(cfg, q, q0, qp, tips_ref, dirs_ref, d_refs)
        return j, g

    bounds = list(zip(cfg.bounds_lo.tolist(), cfg.bounds_hi.tolist()))
    res = minimize(
        lambda q: fun_and_grad(q)[0],
        q0,
        jac=lambda q: fun_and_grad(q)[1],
        method="SLSQP",
        bounds=bounds,
        options={"maxiter": cfg.max_iter, "ftol": cfg.ftol, "disp": False},
    )
    q_opt = np.clip(res.x, cfg.bounds_lo, cfg.bounds_hi)

    # Diagnostics
    _, _, info = evaluate(cfg, q_opt, q0, qp, tips_ref, dirs_ref, d_refs)
    info["iters"] = res.nit
    info["status"] = int(res.status)
    info["d_refs"] = d_refs
    return q_opt, info
