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
    # Per-pair thresholds (index / middle / ring / pinky — pair with thumb).
    # Ring/pinky have bigger baseline distance on DG5F so their sigmoid needs
    # a wider range; otherwise ring/pinky pinches never activate past ~25%.
    pinch_close_ref_m: np.ndarray = field(
        default_factory=lambda: np.array([0.030, 0.030, 0.040, 0.050])
    )
    pinch_far_ref_m: np.ndarray = field(
        default_factory=lambda: np.array([0.100, 0.120, 0.180, 0.220])
    )
    pinch_weight_per_pair: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 1.0, 1.5, 1.8])
    )
    pinch_target_min_m: float = 0.003
    pinch_rescale_k:   float = 0.6
    # Distance residuals are in metres; scale up so (0.02 m)^2 = 4e-4 doesn't
    # get dwarfed by rad^2 prior terms. Internally work in centimetres.
    pinch_distance_scale: float = 100.0
    # Position along each finger's distal phalanx used as the "pinch point"
    # that the pinch term pulls toward. 1.0 = fingertip (tip-to-tip grip);
    # 0.4 = ~40 % along the distal phalanx from the DIP joint (pad grip).
    # Thumb uses the same frac (symmetry). Lower values encourage the thumb
    # to meet the side/pad of the finger rather than the nail tip.
    pinch_finger_frac: float = 1.0


@dataclass
class AObjectiveConfig:
    weights: AObjectiveWeights = field(default_factory=AObjectiveWeights)
    finger_chains: List[FingerChain] = field(default_factory=list)  # len = 5
    bounds_lo: np.ndarray = field(default_factory=lambda: np.zeros(20))
    bounds_hi: np.ndarray = field(default_factory=lambda: np.zeros(20))
    max_iter: int = 4
    ftol: float = 1e-4           # only used by slsqp
    gtol: float = 1e-2           # only used by projgd (early-exit on tiny grad)
    projgd_lr0: float = 0.003    # initial step size for projgd
    solver: str = "projgd"       # "projgd" (fast default) | "slsqp" (legacy)


def _sigmoid_intent(d_ref: float, close: float, far: float) -> float:
    """1.0 when d_ref <= close, 0.0 when d_ref >= far, smooth in between."""
    if d_ref <= close:
        return 1.0
    if d_ref >= far:
        return 0.0
    x = (d_ref - close) / (far - close)
    # cosine ramp, C^1 continuous
    return 0.5 * (1.0 + np.cos(np.pi * x))


def _pinch_target(d_ref: float, close: float, cfg: AObjectiveWeights) -> float:
    """Rescaled target pinch distance. Closes gap when d_ref is small."""
    if d_ref <= close:
        return cfg.pinch_target_min_m
    return cfg.pinch_target_min_m + cfg.pinch_rescale_k * (d_ref - close)


def _pinch_point_and_jac(chain: FingerChain, q: np.ndarray,
                          fk: Dict[str, np.ndarray], frac: float
                          ) -> Tuple[np.ndarray, np.ndarray]:
    """Point = (1-frac)*joint_pos[3] + frac*tip_pos; returns (p, J).

    joint_pos[3] is the DIP-joint anchor; it does NOT depend on q_3, so the
    Jacobian col 3 only has the `frac * d(tip)/dq_3` contribution.
    """
    tip = fk["tip_pos"]
    jp = fk["joint_pos"]
    jR = fk["joint_R"]
    p = (1.0 - frac) * jp[3] + frac * tip
    J = np.zeros((3, 4))
    for i in range(4):
        axis_world = jR[i] @ chain.axes[i]
        d_tip = np.cross(axis_world, tip - jp[i])
        if i < 3:
            d_jp4 = np.cross(axis_world, jp[3] - jp[i])
            J[:, i] = (1.0 - frac) * d_jp4 + frac * d_tip
        else:
            J[:, i] = frac * d_tip
    return p, J


def _fk_all(chains: List[FingerChain], q: np.ndarray,
            pinch_frac: float = 1.0):
    """FK for all 5 fingers; returns list of FK dicts + tip positions + tip
    directions + pinch points. Pinch points interpolate between joint_4 and
    tip according to `pinch_frac` (1.0 = tip, 0.4 = pad-ish).
    """
    fks = []
    tips = np.zeros((5, 3))
    tip_dirs = np.zeros((5, 3))
    pinch_pts = np.zeros((5, 3))
    for i, ch in enumerate(chains):
        fk = fk_finger(ch, q[FINGER_SLOT_SLICES[i]])
        fks.append(fk)
        tips[i] = fk["tip_pos"]
        tip_dirs[i] = fk["tip_dir"]
        if pinch_frac >= 1.0 - 1e-9:
            pinch_pts[i] = fk["tip_pos"]
        else:
            pinch_pts[i] = (1.0 - pinch_frac) * fk["joint_pos"][3] + pinch_frac * fk["tip_pos"]
    return fks, tips, tip_dirs, pinch_pts


def evaluate(cfg: AObjectiveConfig, q: np.ndarray, q0: np.ndarray,
             q_prev: np.ndarray, tips_ref: np.ndarray, dirs_ref: np.ndarray,
             d_refs: np.ndarray):
    """Compute A objective + analytical gradient at q.

    tips_ref / dirs_ref / d_refs are precomputed once per glove frame from q0.
    d_refs is now built from PINCH POINTS of q0 (not tips) so the pinch
    sigmoid intent stays consistent with the pinch point the gradient
    actually drives. For pinch_finger_frac=1.0 pinch points == tips.
    """
    w = cfg.weights
    chains = cfg.finger_chains
    frac = w.pinch_finger_frac

    fks, tips, dirs, pps = _fk_all(chains, q, pinch_frac=frac)

    # Scalar terms
    r_prior = q - q0
    r_vel = q - q_prev
    j_prior = w.prior * float(np.dot(r_prior, r_prior))
    j_vel = w.velocity * float(np.dot(r_vel, r_vel))

    # Orient per finger (still based on tip direction, not pinch point)
    j_orient = 0.0
    for i in range(5):
        j_orient += 1.0 - float(np.dot(dirs[i], dirs_ref[i]))
    j_orient *= w.orient

    # Pinch: thumb pinch point vs finger i=1..4 pinch points.
    j_pinch = 0.0
    pinch_info = []
    s = w.pinch_distance_scale
    # Precompute thumb pinch-point Jacobian once (used for each pair).
    _, Jp_thumb = _pinch_point_and_jac(chains[0], q[FINGER_SLOT_SLICES[0]], fks[0], frac)
    for fi in range(1, 5):
        idx = fi - 1
        v = pps[0] - pps[fi]
        d = float(np.linalg.norm(v)) + 1e-9
        d_ref = float(d_refs[fi])
        close = float(w.pinch_close_ref_m[idx])
        far = float(w.pinch_far_ref_m[idx])
        pair_w = float(w.pinch_weight_per_pair[idx])
        tgt = _pinch_target(d_ref, close, w)
        g = _sigmoid_intent(d_ref, close, far)
        err_s = (d - tgt) * s
        j_pinch += pair_w * g * err_s * err_s
        pinch_info.append((fi, d, tgt, g, v, err_s, pair_w))
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
        grad[sl] += -w.orient * (Jd.T @ dirs_ref[i])

    # pinch grad: obj = w_pinch * pair_w * g * (s * (d - tgt))^2
    for (fi, d, tgt, g, v, err_s, pair_w) in pinch_info:
        _, Jp_f = _pinch_point_and_jac(chains[fi], q[FINGER_SLOT_SLICES[fi]],
                                       fks[fi], frac)
        vn = v / d
        d_err_d_qthumb = vn @ Jp_thumb
        d_err_d_qfi = -vn @ Jp_f
        coef = 2.0 * w.pinch * pair_w * g * s * err_s
        grad[FINGER_SLOT_SLICES[0]] += coef * d_err_d_qthumb
        grad[FINGER_SLOT_SLICES[fi]] += coef * d_err_d_qfi

    return total, grad, {"j_prior": j_prior, "j_vel": j_vel,
                        "j_orient": j_orient, "j_pinch": j_pinch,
                        "tips": tips, "dirs": dirs, "pinch_pts": pps}


def _solve_projgd(cfg: AObjectiveConfig, q0: np.ndarray, qp: np.ndarray,
                  tips_ref: np.ndarray, dirs_ref: np.ndarray,
                  d_refs: np.ndarray) -> Tuple[np.ndarray, int]:
    """Projected gradient descent with Armijo-lite line search.

    Empirically converges to ~5 mm thumb-finger pinch gap in 4-6 iterations
    when the problem is well warm-started (which ergo_q0 always gives).
    scipy SLSQP has enough Python overhead per call that we routinely
    topped out at 10-20 Hz; rolling our own projection + step halving gets
    the loop below 25 ms typically.
    """
    q = q0.copy()
    prev_j = np.inf
    lr = cfg.projgd_lr0
    for it in range(cfg.max_iter):
        j, g, _ = evaluate(cfg, q, q0, qp, tips_ref, dirs_ref, d_refs)
        if j > prev_j:
            lr *= 0.5
        prev_j = j
        if np.linalg.norm(g) < cfg.gtol:
            return q, it + 1
        q = np.clip(q - lr * g, cfg.bounds_lo, cfg.bounds_hi)
    return q, cfg.max_iter


def _solve_slsqp(cfg: AObjectiveConfig, q0: np.ndarray, qp: np.ndarray,
                 tips_ref: np.ndarray, dirs_ref: np.ndarray,
                 d_refs: np.ndarray) -> Tuple[np.ndarray, int]:
    """Legacy SLSQP. Accurate but often >100 ms on hard frames. Keep as
    a reference solver for comparison / offline tuning."""
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
    return np.clip(res.x, cfg.bounds_lo, cfg.bounds_hi), int(res.nit)


def solve_ik(cfg: AObjectiveConfig, ergo_q0: np.ndarray,
             q_prev: Optional[np.ndarray] = None) -> Tuple[np.ndarray, dict]:
    """Run the configured solver from warm-start q0.

    ergo_q0: (20,) ergo-mapped joint angles (warm start + prior reference).
    q_prev:  (20,) last commanded angles (velocity reg); defaults to q0 if None.
    Returns (q_opt, info_dict).
    """
    q0 = np.clip(ergo_q0, cfg.bounds_lo, cfg.bounds_hi)
    qp = q0 if q_prev is None else np.clip(q_prev, cfg.bounds_lo, cfg.bounds_hi)

    _, tips_ref, dirs_ref, pps_ref = _fk_all(
        cfg.finger_chains, q0, pinch_frac=cfg.weights.pinch_finger_frac)
    d_refs = np.zeros(5)
    for fi in range(1, 5):
        d_refs[fi] = float(np.linalg.norm(pps_ref[0] - pps_ref[fi]))

    if cfg.solver == "slsqp":
        q_opt, iters = _solve_slsqp(cfg, q0, qp, tips_ref, dirs_ref, d_refs)
    else:
        q_opt, iters = _solve_projgd(cfg, q0, qp, tips_ref, dirs_ref, d_refs)

    _, _, info = evaluate(cfg, q_opt, q0, qp, tips_ref, dirs_ref, d_refs)
    info["iters"] = iters
    info["d_refs"] = d_refs
    return q_opt, info
