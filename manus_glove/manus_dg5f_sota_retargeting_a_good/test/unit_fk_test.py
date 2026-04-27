"""FK + analytical Jacobian sanity checks against numerical differentiation."""
import os
import sys
import unittest

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(THIS)
sys.path.insert(0, ROOT)

from manus_dg5f_sota_retargeting_a_good.urdf_fk import (  # noqa: E402
    build_finger_chain, fk_finger, jacobian_tip_dir, jacobian_tip_pos, parse_urdf
)

URDF_RIGHT = os.path.abspath(os.path.join(
    THIS, "..", "..", "..", "dg", "dg5f_ros2", "dg5f_description",
    "urdf", "dg5f_right.urdf"
))
URDF_LEFT = URDF_RIGHT.replace("dg5f_right.urdf", "dg5f_left.urdf")


def _num_jac(fn, q, eps=1e-6):
    J = np.zeros((3, 4))
    for i in range(4):
        dq = np.zeros(4)
        dq[i] = eps
        J[:, i] = (fn(q + dq) - fn(q - dq)) / (2 * eps)
    return J


class UrdfFkTest(unittest.TestCase):
    def _chain(self, urdf_path, prefix, finger):
        joints = parse_urdf(urdf_path)
        jn = [f"{prefix}_dg_{finger}_{k}" for k in range(1, 5)]
        tip = f"{prefix}_dg_{finger}_tip"
        palm = f"{prefix[0]}l_dg_palm"
        return build_finger_chain(joints, jn, tip, palm)

    def test_all_right_fingers_jac(self):
        for f in (1, 2, 3, 4, 5):
            chain = self._chain(URDF_RIGHT, "rj", f)
            rng = np.random.default_rng(f)
            for _ in range(3):
                q = rng.uniform(chain.limits_lo, chain.limits_hi)
                fk = fk_finger(chain, q)
                Jp_a = jacobian_tip_pos(chain, q, fk)
                Jp_n = _num_jac(lambda qq: fk_finger(chain, qq)["tip_pos"], q)
                self.assertLess(np.abs(Jp_a - Jp_n).max(), 1e-6,
                                msg=f"right finger {f} tip_pos jac off")

                Jd_a = jacobian_tip_dir(chain, q, fk)
                Jd_n = _num_jac(lambda qq: fk_finger(chain, qq)["tip_dir"], q)
                self.assertLess(np.abs(Jd_a - Jd_n).max(), 1e-6,
                                msg=f"right finger {f} tip_dir jac off")

    def test_left_chain_builds(self):
        chain = self._chain(URDF_LEFT, "lj", 2)
        self.assertEqual(chain.origins.shape, (4, 3))
        # limits are reasonable
        self.assertTrue(np.all(chain.limits_lo < chain.limits_hi))


if __name__ == "__main__":
    unittest.main()
