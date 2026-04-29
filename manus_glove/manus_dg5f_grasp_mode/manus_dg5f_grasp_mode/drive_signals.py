"""Compute drive scalars (curl ratios) from a Manus glove ergo frame.

A drive scalar is a clamped float in [0,1] that goes from 0 (finger
fully extended) to 1 (finger fully curled). Variable_axes in the
grasp_modes yaml reference these by name (`thumb_curl`, `index_curl`,
`hand_curl`, ...) — the wrapper node looks up the float here.

Curl is the sum of MCP/PIP/DIP stretch (deg) divided by 210 deg
(70 per joint, an empirical "fully closed" sum). Spread is ignored
on purpose — flexion is what users intuitively control with grip
intent.
"""
from __future__ import annotations

from typing import Dict


_CURL_FULL_DEG = 210.0  # 3 joints * ~70 deg each


def _strip_side(ergo: Dict[str, float]) -> Dict[str, float]:
    """Manus driver sometimes prefixes ergo names with 'Right'/'Left'."""
    for prefix in ("Right", "Left"):
        if any(k.startswith(prefix) for k in ergo):
            return {(k[len(prefix):] if k.startswith(prefix) else k): v
                    for k, v in ergo.items()}
    return dict(ergo)


def _finger_curl(e: Dict[str, float], finger: str) -> float:
    s = (
        float(e.get(f"{finger}MCPStretch", 0.0))
        + float(e.get(f"{finger}PIPStretch", 0.0))
        + float(e.get(f"{finger}DIPStretch", 0.0))
    )
    return max(0.0, min(1.0, s / _CURL_FULL_DEG))


def compute_drives(ergo: Dict[str, float]) -> Dict[str, float]:
    """Return the named drive scalars used by grasp_modes variable_axes."""
    e = _strip_side(ergo)
    fingers = {
        "thumb":  "Thumb",
        "index":  "Index",
        "middle": "Middle",
        "ring":   "Ring",
        "pinky":  "Pinky",
    }
    drives: Dict[str, float] = {}
    for short, long in fingers.items():
        drives[f"{short}_curl"] = _finger_curl(e, long)
    drives["hand_curl"] = sum(
        drives[f"{f}_curl"] for f in fingers
    ) / len(fingers)
    return drives
