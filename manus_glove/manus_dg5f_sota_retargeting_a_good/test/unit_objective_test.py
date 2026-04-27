"""Analytical gradient of A objective vs. numerical."""
import os
import sys
import unittest

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(THIS)
sys.path.insert(0, ROOT)

from manus_dg5f_sota_retargeting_a_good.urdf_fk import (  # noqa: E402
    build_finger_chain, parse_urdf
)
from manus_dg5f_sota_retargeting_a_good.retarget_ik import (  # noqa: E402
    AObjectiveConfig, AObjectiveWeights, evaluate,
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
        tip = f"rj_dg_{f}_tip"
        chains.append(build_finger_chain(joints, jn, tip, "rl_dg_palm"))
    bounds_lo = np.concatenate([ch.limits_lo for ch in chains])
    bounds_hi = np.concatenate([ch.limits_hi for ch in chains])
    return AObjectiveConfig(
        weights=AObjectiveWeights(),
        finger_chains=chains,
        bounds_lo=bounds_lo,
        bounds_hi=bounds_hi,
    )


def _num_grad(fn, q, eps=1e-6):
    g = np.zeros_like(q)
    for i in range(q.size):
        dq = np.zeros_like(q); dq[i] = eps
        g[i] = (fn(q + dq) - fn(q - dq)) / (2 * eps)
    return g


class ObjectiveGradTest(unittest.TestCase):
    def test_grad_matches_numerical(self):
        cfg = _build_cfg()
        rng = np.random.default_rng(0)
        q0 = rng.uniform(cfg.bounds_lo * 0.6, cfg.bounds_hi * 0.6)
        q_prev = q0.copy()

        # reference FK for pinch/orient terms
        from manus_dg5f_sota_retargeting_a_good.retarget_ik import _fk_all  # type: ignore
        _, tips_ref, dirs_ref = _fk_all(cfg.finger_chains, q0)
        d_refs = np.zeros(5)
        for fi in range(1, 5):
            d_refs[fi] = float(np.linalg.norm(tips_ref[0] - tips_ref[fi]))

        q = q0 + rng.normal(0, 0.05, 20)
        q = np.clip(q, cfg.bounds_lo, cfg.bounds_hi)

        j, ga, _ = evaluate(cfg, q, q0, q_prev, tips_ref, dirs_ref, d_refs)

        def scal(qq):
            jq, _, _ = evaluate(cfg, qq, q0, q_prev, tips_ref, dirs_ref, d_refs)
            return jq

        gn = _num_grad(scal, q)
        err = np.abs(ga - gn).max()
        # tolerance loose-ish because of sqrt in pinch distance
        self.assertLess(err, 1e-4, msg=f"grad err {err}")


if __name__ == "__main__":
    unittest.main()
