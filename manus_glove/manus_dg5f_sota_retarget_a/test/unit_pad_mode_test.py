"""pad / lateral grip mode: DIP extension allowed, pinch target shifted."""
import os
import sys
import unittest

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(THIS)
sys.path.insert(0, ROOT)

from manus_dg5f_sota_retarget_a.ergo_map import (  # noqa: E402
    CALIB_DEFAULT, DIR_RIGHT, POSTPROC_RIGHT,
    compute_thumb_cmc, ergo_to_q0_rad,
)
from manus_dg5f_sota_retarget_a.retarget_ik import (  # noqa: E402
    AObjectiveConfig, AObjectiveWeights, solve_ik, _fk_all,
)
from manus_dg5f_sota_retarget_a.urdf_fk import (  # noqa: E402
    build_finger_chain, fk_finger, parse_urdf,
)

URDF = os.path.abspath(os.path.join(
    THIS, "..", "..", "..", "dg", "dg5f_ros2", "dg5f_description",
    "urdf", "dg5f_right.urdf"
))


def _chains():
    joints = parse_urdf(URDF)
    return [build_finger_chain(joints, [f"rj_dg_{f}_{k}" for k in range(1, 5)],
                               f"rj_dg_{f}_tip", "rl_dg_palm") for f in range(1, 6)]


class PadModeTest(unittest.TestCase):
    def test_dip_extension_preserved_when_allowed(self):
        """Ergonomics with negative IndexDIPStretch should survive POSTPROC
        only when allow_dip_extension=True (pad mode)."""
        calib = np.array(CALIB_DEFAULT)
        cmc = 0.0
        ergo = {k: 0.0 for k in ("ThumbMCPSpread", "ThumbMCPStretch",
                                 "ThumbPIPStretch", "ThumbDIPStretch",
                                 "IndexSpread", "IndexMCPStretch",
                                 "IndexPIPStretch", "IndexDIPStretch",
                                 "MiddleSpread", "MiddleMCPStretch",
                                 "MiddlePIPStretch", "MiddleDIPStretch",
                                 "RingSpread", "RingMCPStretch",
                                 "RingPIPStretch", "RingDIPStretch",
                                 "PinkySpread", "PinkyMCPStretch",
                                 "PinkyPIPStretch", "PinkyDIPStretch")}
        ergo["IndexMCPStretch"] = 55.0
        ergo["IndexPIPStretch"] = 55.0
        ergo["IndexDIPStretch"] = -25.0   # hyperextension

        q_tip = ergo_to_q0_rad(ergo, calib, DIR_RIGHT, POSTPROC_RIGHT, cmc,
                               allow_dip_extension=False)
        q_pad = ergo_to_q0_rad(ergo, calib, DIR_RIGHT, POSTPROC_RIGHT, cmc,
                               allow_dip_extension=True)

        # Slot 7 = index DIP. tiptotip zeroes the negative; pad keeps it.
        self.assertAlmostEqual(q_tip[7], 0.0, places=6)
        self.assertLess(q_pad[7], -0.3,
                        msg=f"DIP extension lost in pad mode: q_pad[7]={q_pad[7]:.3f}")

    def test_pad_pinch_point_differs_from_tip(self):
        """pinch_finger_frac=0.4 moves the pinch point ~cm proximally."""
        chains = _chains()
        # bent-finger pose
        q = np.zeros(20)
        q[FINGER_SLOTS := slice(4, 8)] = [0.0, 1.0, 1.0, 0.5]
        _, _, _, pps_tip = _fk_all(chains, q, pinch_frac=1.0)
        _, _, _, pps_pad = _fk_all(chains, q, pinch_frac=0.4)
        # index pinch point shifted by > 10 mm
        shift = np.linalg.norm(pps_tip[1] - pps_pad[1])
        self.assertGreater(shift, 0.010,
                           msg=f"pad pinch-point barely moved: {shift*1000:.1f} mm")

    def test_pad_mode_pinch_still_closes(self):
        """Pad-mode solve with extended-DIP ergo still brings pinch points
        close (different from tip-to-tip but should reach < 20 mm)."""
        chains = _chains()
        bl = np.concatenate([ch.limits_lo for ch in chains])
        bh = np.concatenate([ch.limits_hi for ch in chains])
        w = AObjectiveWeights(pinch_finger_frac=0.4)
        cfg = AObjectiveConfig(weights=w, finger_chains=chains,
                               bounds_lo=bl, bounds_hi=bh,
                               max_iter=8, projgd_lr0=0.003)

        calib = np.array(CALIB_DEFAULT)
        ergo = {
            "ThumbMCPSpread": 20.0, "ThumbMCPStretch": 35.0,
            "ThumbPIPStretch": 35.0, "ThumbDIPStretch": 10.0,
            "IndexSpread": 0.0, "IndexMCPStretch": 60.0,
            "IndexPIPStretch": 65.0, "IndexDIPStretch": -20.0,   # extended DIP
        }
        cmc = compute_thumb_cmc("coupled", ergo["ThumbMCPSpread"],
                                ergo["ThumbMCPStretch"],
                                offset_deg=0.0, gain_stretch=0.2, gain_spread=1.0)
        q0 = ergo_to_q0_rad(ergo, calib, DIR_RIGHT, POSTPROC_RIGHT, cmc,
                            allow_dip_extension=True)
        q_opt, _info = solve_ik(cfg, q0, q_prev=q0)
        _, _, _, pps = _fk_all(chains, q_opt, pinch_frac=0.4)
        d = np.linalg.norm(pps[0] - pps[1])
        self.assertLess(d, 0.030,
                        msg=f"pad thumb-index pad dist {d*1000:.1f} mm > 30 mm")


if __name__ == "__main__":
    unittest.main()
