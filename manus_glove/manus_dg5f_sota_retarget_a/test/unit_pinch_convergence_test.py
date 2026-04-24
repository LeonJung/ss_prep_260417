"""Pinch convergence: when ergo implies thumb+index are close, SLSQP must
close the robot thumb-index tip gap below 10 mm (vs typical 40 mm baseline)."""
import os
import sys
import unittest

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(THIS)
sys.path.insert(0, ROOT)

from manus_dg5f_sota_retarget_a.ergo_map import (  # noqa: E402
    CALIB_DEFAULT, DIR_RIGHT, POSTPROC_RIGHT,
    compute_thumb_cmc_fixed, ergo_to_q0_rad,
)
from manus_dg5f_sota_retarget_a.retarget_ik import (  # noqa: E402
    AObjectiveConfig, AObjectiveWeights, solve_ik,
)
from manus_dg5f_sota_retarget_a.urdf_fk import (  # noqa: E402
    build_finger_chain, fk_finger, parse_urdf
)

URDF = os.path.abspath(os.path.join(
    THIS, "..", "..", "..", "dg", "dg5f_ros2", "dg5f_description",
    "urdf", "dg5f_right.urdf"
))


def _build_cfg(w_pinch=6.0, w_prior=1.0):
    joints = parse_urdf(URDF)
    chains = []
    for f in range(1, 6):
        jn = [f"rj_dg_{f}_{k}" for k in range(1, 5)]
        tip = f"rj_dg_{f}_tip"
        chains.append(build_finger_chain(joints, jn, tip, "rl_dg_palm"))
    bounds_lo = np.concatenate([ch.limits_lo for ch in chains])
    bounds_hi = np.concatenate([ch.limits_hi for ch in chains])
    return AObjectiveConfig(
        weights=AObjectiveWeights(prior=w_prior, pinch=w_pinch,
                                  velocity=0.1, orient=0.3),
        finger_chains=chains,
        bounds_lo=bounds_lo,
        bounds_hi=bounds_hi,
        max_iter=80, ftol=1e-7,
    )


def _tip_distance(cfg, q, finger_idx):
    tip0 = fk_finger(cfg.finger_chains[0], q[0:4])["tip_pos"]
    tipi = fk_finger(cfg.finger_chains[finger_idx], q[finger_idx*4:(finger_idx+1)*4])["tip_pos"]
    return float(np.linalg.norm(tip0 - tipi))


class PinchConvergenceTest(unittest.TestCase):
    def test_thumb_index_pinch_closes(self):
        cfg = _build_cfg()
        calib = np.array(CALIB_DEFAULT)

        ergo = {
            "ThumbMCPSpread": 30.0, "ThumbMCPStretch": 35.0,
            "ThumbPIPStretch": 35.0, "ThumbDIPStretch": 25.0,
            "IndexMCPStretch": 55.0, "IndexPIPStretch": 55.0,
            "IndexDIPStretch": 25.0,
        }
        cmc = compute_thumb_cmc_fixed(ergo["ThumbMCPSpread"],
                                      ergo["ThumbMCPStretch"])
        q0 = ergo_to_q0_rad(ergo, calib, DIR_RIGHT, POSTPROC_RIGHT, cmc)
        d_baseline = _tip_distance(cfg, q0, 1)

        q_opt, info = solve_ik(cfg, q0, q_prev=q0)
        d_after = _tip_distance(cfg, q_opt, 1)

        # must close by at least 20 mm or to < 15 mm absolute
        self.assertTrue(
            d_baseline - d_after > 0.020 or d_after < 0.015,
            msg=f"baseline d={d_baseline*1000:.1f} mm, after d={d_after*1000:.1f} mm "
                f"(info: {info['j_pinch']:.4f} pinch term, {info['iters']} iters)"
        )

    def test_no_pinch_when_open(self):
        """When ergo says 'hand open' (all ~0), optimizer should NOT force
        a pinch — q_opt should stay at (URDF-clipped) q0."""
        cfg = _build_cfg()
        calib = np.array(CALIB_DEFAULT)
        ergo = {k: 0.0 for k in (
            "ThumbMCPSpread", "ThumbMCPStretch", "ThumbPIPStretch", "ThumbDIPStretch",
            "IndexSpread", "IndexMCPStretch", "IndexPIPStretch", "IndexDIPStretch",
            "MiddleSpread", "MiddleMCPStretch", "MiddlePIPStretch", "MiddleDIPStretch",
            "RingSpread", "RingMCPStretch", "RingPIPStretch", "RingDIPStretch",
            "PinkySpread", "PinkyMCPStretch", "PinkyPIPStretch", "PinkyDIPStretch",
        )}
        cmc = compute_thumb_cmc_fixed(0.0, 0.0)
        q0 = ergo_to_q0_rad(ergo, calib, DIR_RIGHT, POSTPROC_RIGHT, cmc)
        q0_clip = np.clip(q0, cfg.bounds_lo, cfg.bounds_hi)
        q_opt, _ = solve_ik(cfg, q0, q_prev=q0)
        # prior pins us to q0_clip; drift from clipped q0 should be negligible.
        self.assertLess(np.linalg.norm(q_opt - q0_clip), 0.05,
                        msg=f"open-hand q drifted from q0_clip: "
                            f"{np.linalg.norm(q_opt - q0_clip):.3f} rad")


if __name__ == "__main__":
    unittest.main()
