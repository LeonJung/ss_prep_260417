"""Lite calibration math.

Tunes:
  - calib[20]: per-joint scaling so open->q0[i]≈0 and fist->q0[i]≈0.85*upper.
  - pinch_close_ref_m[4], pinch_far_ref_m[4]: from observed open + tip-pinch
    FK distances per (thumb, finger) pair.

NOT tuned (left for manual yaml edit):
  - thumb_cmc_* (mode/gains/offset/fixed)
  - dir_sign[20]
  - pinch_target_min_m / pinch_rescale_k / pinch_finger_frac
  - solver/weights
"""
from __future__ import annotations

import copy
from typing import Any, Dict, List

import numpy as np

from manus_dg5f_sota_retarget_a.ergo_map import (
    DIR_LEFT, DIR_RIGHT, ERGO_KEYS, POSTPROC_LEFT, POSTPROC_RIGHT,
    compute_thumb_cmc_fixed, ergo_to_q0_rad,
)
from manus_dg5f_sota_retarget_a.urdf_fk import (
    build_finger_chain, fk_finger, parse_urdf,
)


# Open: target q0 ≈ 0 for non-CMC slots. Fist: target ≈ 85% of upper limit
# for the "main" joints (MCP/PIP/DIP); abduction slots stay near 0.
# slot indices: 0..3 thumb, 4..7 index, 8..11 middle, 12..15 ring, 16..19 pinky.
# Within each finger: [abduction(spread), MCP, PIP, DIP].
ABDUCTION_SLOTS = (0, 4, 8, 12, 16)
TARGET_FRAC_FIST = 0.85   # fraction of urdf upper limit at fist

CALIB_MIN = 0.4
CALIB_MAX = 3.0


def _build_chains(urdf_path: str, side: str):
    joints = parse_urdf(urdf_path)
    prefix = "rj" if side == "right" else "lj"
    palm = f"{prefix[0]}l_dg_palm"
    chains = []
    for f in range(1, 6):
        jn = [f"{prefix}_dg_{f}_{k}" for k in range(1, 5)]
        tip = f"{prefix}_dg_{f}_tip"
        chains.append(build_finger_chain(joints, jn, tip, palm))
    return chains, [(ch.limits_lo, ch.limits_hi) for ch in chains]


def _ergo_dict_strip(ergo: Dict[str, float]) -> Dict[str, float]:
    """Strip 'Right'/'Left' prefix (real Manus driver pattern)."""
    out = dict(ergo)
    for prefix in ("Right", "Left"):
        if any(k.startswith(prefix) for k in out):
            out = {(k[len(prefix):] if k.startswith(prefix) else k): v
                   for k, v in out.items()}
            break
    return out


def _ergo_to_q0(ergo: Dict[str, float], side: str, cfg: Dict[str, Any]) -> np.ndarray:
    calib = np.asarray(cfg["calib"], dtype=float)
    dir_sign = np.asarray(cfg["dir_sign"], dtype=float)
    postproc = POSTPROC_LEFT if side == "left" else POSTPROC_RIGHT
    cmc_offset = float(cfg.get("thumb_cmc_offset_deg", 58.5))
    cmc_gs = float(cfg.get("thumb_cmc_gain_stretch", 1.0))
    cmc_gp = float(cfg.get("thumb_cmc_gain_spread", 0.0))
    e = _ergo_dict_strip(ergo)
    cmc = compute_thumb_cmc_fixed(
        spread_deg=float(e.get("ThumbMCPSpread", 0.0)),
        stretch_deg=float(e.get("ThumbMCPStretch", 0.0)),
        offset_deg=cmc_offset, gain_stretch=cmc_gs, gain_spread=cmc_gp,
    )
    return ergo_to_q0_rad(e, calib, dir_sign, postproc, cmc)


def _tip_distance(chains, q0: np.ndarray, finger_idx: int) -> float:
    """fingertip-to-fingertip distance between thumb (slot 0..3) and the
    given finger (slot finger_idx*4 .. finger_idx*4+4)."""
    t0 = fk_finger(chains[0], q0[0:4])["tip_pos"]
    ti = fk_finger(chains[finger_idx], q0[finger_idx * 4:(finger_idx + 1) * 4])["tip_pos"]
    return float(np.linalg.norm(t0 - ti))


def tune(cfg: Dict[str, Any], captures: Dict[str, Dict[str, float]],
         urdf_path: str, side: str,
         tip_target_m: float = 0.001, pad_target_m: float = 0.001,
         verbose: bool = True) -> Dict[str, Any]:
    """Return a NEW cfg dict with calib[20] + pinch sigmoid bounds adjusted.

    captures: dict from pose name (open/fist/tip_<finger>/pad_<finger>) to
              the averaged ergo dict for that pose.
    side    : "right" | "left"
    """
    new_cfg = copy.deepcopy(cfg)
    chains, limits = _build_chains(urdf_path, side)
    upper = np.concatenate([hi for (_, hi) in limits])
    # lower = np.concatenate([lo for (lo, _) in limits])

    if "open" not in captures or "fist" not in captures:
        raise ValueError("need at least 'open' and 'fist' captures")

    q0_open = _ergo_to_q0(captures["open"], side, cfg)
    q0_fist = _ergo_to_q0(captures["fist"], side, cfg)

    if verbose:
        print(f"  q0_open thumb+idx: {q0_open[:8].round(3).tolist()}")
        print(f"  q0_fist thumb+idx: {q0_fist[:8].round(3).tolist()}")

    # --- per-joint calib adjustment ---
    new_calib = list(map(float, cfg["calib"]))
    for i in range(20):
        if i == 0:
            continue  # thumb CMC handled by mode/gains
        if i in ABDUCTION_SLOTS:
            # spread joints: target stays ~0 in both poses; skip auto-tune
            continue
        delta = q0_fist[i] - q0_open[i]
        if abs(delta) < 0.10:
            # joint barely moves — leave calib alone
            continue
        target_open = 0.0
        target_fist = TARGET_FRAC_FIST * upper[i]
        target_delta = target_fist - target_open
        scale = target_delta / delta
        # Adjustment is a multiplicative factor on the existing calib.
        new_val = new_calib[i] * scale
        new_val = float(np.clip(new_val, CALIB_MIN, CALIB_MAX))
        new_calib[i] = new_val
    new_cfg["calib"] = new_calib

    # --- pinch sigmoid bounds[4] ---
    finger_names = ["index", "middle", "ring", "pinky"]
    cur_close = list(np.atleast_1d(np.asarray(
        cfg.get("pinch_close_ref_m", [0.030] * 4), dtype=float)))
    cur_far = list(np.atleast_1d(np.asarray(
        cfg.get("pinch_far_ref_m", [0.080] * 4), dtype=float)))
    if len(cur_close) == 1:
        cur_close = cur_close * 4
    if len(cur_far) == 1:
        cur_far = cur_far * 4

    new_close = list(cur_close)
    new_far = list(cur_far)
    for fi, name in enumerate(finger_names):
        pose_name = f"tip_{name}"
        if pose_name not in captures:
            continue
        # Use NEW calib for FK (so the same yaml we're about to write reflects
        # the corrected per-joint scale).
        new_cfg_for_fk = dict(new_cfg)
        q0_pinch = _ergo_to_q0(captures[pose_name], side, new_cfg_for_fk)
        d_pinch = _tip_distance(chains, q0_pinch, fi + 1)
        d_open = _tip_distance(chains, _ergo_to_q0(captures["open"], side, new_cfg_for_fk), fi + 1)
        # close_ref slightly above the pinch-baseline so sigmoid hits 1 there.
        new_close[fi] = round(max(d_pinch * 1.05, 0.020), 4)
        # far_ref well below the open-baseline so the sigmoid is 0 at rest.
        new_far[fi] = round(max(d_open * 0.85, new_close[fi] + 0.020), 4)
        if verbose:
            print(f"  pinch tip_{name}: d_open={d_open*1000:.1f} mm, "
                  f"d_pinch={d_pinch*1000:.1f} mm -> "
                  f"close={new_close[fi]*1000:.1f} far={new_far[fi]*1000:.1f}")
    new_cfg["pinch_close_ref_m"] = new_close
    new_cfg["pinch_far_ref_m"] = new_far

    # Mention the user-targeted pinch closure but don't bake into close_ref —
    # `pinch_target_min_m` is a separate yaml param. Set it to the requested
    # 1 mm target if smaller than current.
    cur_target_min = float(cfg.get("pinch_target_min_m", 0.003))
    new_cfg["pinch_target_min_m"] = float(min(cur_target_min, tip_target_m))
    if verbose:
        print(f"  pinch_target_min_m: {cur_target_min*1000:.1f} -> "
              f"{new_cfg['pinch_target_min_m']*1000:.1f} mm")

    return new_cfg
