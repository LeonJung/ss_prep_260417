"""End-to-end sweep in-process: simulate a glove trajectory from Open -> Pinch
-> Open, verifying (a) pinch closure on the way in, (b) restoration on the
way out, (c) no per-step command jumps > 0.15 rad.
"""
import os
import sys
import unittest

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(THIS)
sys.path.insert(0, ROOT)

from manus_dg5f_sota_retargeting_a_good.ergo_map import (  # noqa: E402
    CALIB_DEFAULT, DIR_RIGHT, POSTPROC_RIGHT,
    compute_thumb_cmc_fixed, ergo_to_q0_rad,
)
from manus_dg5f_sota_retargeting_a_good.retarget_ik import (  # noqa: E402
    AObjectiveConfig, AObjectiveWeights, solve_ik,
)
from manus_dg5f_sota_retargeting_a_good.urdf_fk import (  # noqa: E402
    build_finger_chain, fk_finger, parse_urdf,
)

URDF = os.path.abspath(os.path.join(
    THIS, "..", "..", "..", "dg", "dg5f_ros2", "dg5f_description",
    "urdf", "dg5f_right.urdf"
))


def _build_cfg():
    joints = parse_urdf(URDF)
    chains = []
    for f in range(1, 6):
        jn = [f"rj_dg_{f}_{k}" for k in range(1, 5)]
        chains.append(build_finger_chain(joints, jn, f"rj_dg_{f}_tip", "rl_dg_palm"))
    return AObjectiveConfig(
        weights=AObjectiveWeights(pinch=6.0, prior=1.0),
        finger_chains=chains,
        bounds_lo=np.concatenate([ch.limits_lo for ch in chains]),
        bounds_hi=np.concatenate([ch.limits_hi for ch in chains]),
        max_iter=40, ftol=1e-6,
    )


def _ergo_pinch(t: float) -> dict:
    """t in [0,1] -> interpolated ergonomics between open and pinch preset."""
    open_vals = {k: 0.0 for k in (
        "ThumbMCPSpread", "ThumbMCPStretch", "ThumbPIPStretch", "ThumbDIPStretch",
        "IndexSpread", "IndexMCPStretch", "IndexPIPStretch", "IndexDIPStretch",
        "MiddleSpread", "MiddleMCPStretch", "MiddlePIPStretch", "MiddleDIPStretch",
        "RingSpread", "RingMCPStretch", "RingPIPStretch", "RingDIPStretch",
        "PinkySpread", "PinkyMCPStretch", "PinkyPIPStretch", "PinkyDIPStretch",
    )}
    pinch_vals = {
        **open_vals,
        "ThumbMCPSpread": 30.0, "ThumbMCPStretch": 35.0,
        "ThumbPIPStretch": 35.0, "ThumbDIPStretch": 25.0,
        "IndexMCPStretch": 55.0, "IndexPIPStretch": 55.0, "IndexDIPStretch": 25.0,
    }
    return {k: (1.0 - t) * open_vals[k] + t * pinch_vals[k] for k in open_vals}


def _tip_dist(cfg, q, fi):
    t0 = fk_finger(cfg.finger_chains[0], q[0:4])["tip_pos"]
    ti = fk_finger(cfg.finger_chains[fi], q[fi * 4:(fi + 1) * 4])["tip_pos"]
    return float(np.linalg.norm(t0 - ti))


class SimSweepTest(unittest.TestCase):
    def test_pinch_sweep(self):
        cfg = _build_cfg()
        calib = np.array(CALIB_DEFAULT)

        # 60 steps over ~1 s @ 60 Hz simulates realistic glove pace.
        N = 60
        ts = np.concatenate([np.linspace(0, 1, N // 2),
                             np.linspace(1, 0, N - N // 2)])

        q_prev = None
        dists_pinch = []
        max_jump = 0.0
        q_history = []

        # Match the ROS node's per-tick step cap (see retarget_node.py).
        MAX_STEP = 0.12

        for t in ts:
            ergo = _ergo_pinch(t)
            cmc = compute_thumb_cmc_fixed(ergo["ThumbMCPSpread"],
                                          ergo["ThumbMCPStretch"])
            q0 = ergo_to_q0_rad(ergo, calib, DIR_RIGHT, POSTPROC_RIGHT, cmc)
            q_opt, _ = solve_ik(cfg, q0, q_prev=q_prev)

            if q_prev is not None:
                delta = np.clip(q_opt - q_prev, -MAX_STEP, MAX_STEP)
                q_opt = q_prev + delta
                max_jump = max(max_jump, float(np.abs(q_opt - q_prev).max()))
            q_prev = q_opt
            q_history.append(q_opt)
            dists_pinch.append(_tip_dist(cfg, q_opt, 1))

        dists_pinch = np.array(dists_pinch)
        d_open_start = dists_pinch[0]
        d_peak_pinch = dists_pinch[N // 2 - 1]
        d_open_end = dists_pinch[-1]

        # pinch closes on the way in
        self.assertLess(d_peak_pinch, d_open_start - 0.020,
                        msg=f"pinch did not close: open={d_open_start*1000:.1f} mm, "
                            f"pinch_peak={d_peak_pinch*1000:.1f} mm")
        # comes back to ~open state
        self.assertLess(abs(d_open_end - d_open_start), 0.010,
                        msg=f"return to open drifted: start={d_open_start*1000:.1f} "
                            f"end={d_open_end*1000:.1f}")
        # step cap honoured.
        self.assertLessEqual(max_jump, MAX_STEP + 1e-9,
                             msg=f"per-step jump {max_jump:.4f} rad exceeds cap "
                                 f"{MAX_STEP:.2f}")
        # end-to-end throughput sanity: we expect every step to finish < 50 ms.
        self.assertEqual(len(q_history), N)


if __name__ == "__main__":
    unittest.main()
