"""Compute drive scalars (curl ratios) from a Manus glove ergo frame.

A drive scalar is a clamped float in [0,1] — 0 when the finger is
fully extended, 1 when fully curled. Variable_axes in the
grasp_modes yaml reference these by name (`thumb_curl`, `index_curl`,
`hand_curl`, ...) — the wrapper node looks up the float here.

Curl = (MCPStretch + PIPStretch + DIPStretch) (deg) / curl_full_deg.
Spread is ignored on purpose — flexion is what users intuitively
control with grip intent.

Per-finger `curl_full_deg`:
  - thumb defaults to 60 deg: the Manus thumb's usable stretch sum
    at "full flex" was empirically ~50–60 deg with our user's glove
    — using 210 (or even 100) here mapped a maxed thumb to only
    ~0.3–0.5 drive, so the DG5F barely moved past halfway. 60 deg
    leaves a small headroom so a comfortably curled thumb already
    saturates to 1.0.
  - long fingers default to 210 deg (3 × 70 deg), tuned to typical
    Manus driver output for a tight fist.
Override per-finger via the wrapper node's `curl_full_deg` ROS param
(5-element list: thumb, index, middle, ring, pinky).
"""
from __future__ import annotations

from typing import Dict, Iterable, Optional, Sequence


# (thumb, index, middle, ring, pinky)
DEFAULT_CURL_FULL_DEG: Sequence[float] = (40.0, 210.0, 210.0, 210.0, 210.0)


def _strip_side(ergo: Dict[str, float]) -> Dict[str, float]:
    """Manus driver sometimes prefixes ergo names with 'Right'/'Left'."""
    for prefix in ("Right", "Left"):
        if any(k.startswith(prefix) for k in ergo):
            return {(k[len(prefix):] if k.startswith(prefix) else k): v
                    for k, v in ergo.items()}
    return dict(ergo)


def _finger_stretch_deg(e: Dict[str, float], finger: str) -> float:
    return (
        float(e.get(f"{finger}MCPStretch", 0.0))
        + float(e.get(f"{finger}PIPStretch", 0.0))
        + float(e.get(f"{finger}DIPStretch", 0.0))
    )


_FINGERS = (
    ("thumb",  "Thumb"),
    ("index",  "Index"),
    ("middle", "Middle"),
    ("ring",   "Ring"),
    ("pinky",  "Pinky"),
)


def compute_drives(ergo: Dict[str, float],
                   curl_full_deg: Optional[Iterable[float]] = None
                   ) -> Dict[str, float]:
    """Return the named drive scalars used by grasp_modes variable_axes.

    `curl_full_deg`: optional 5-element override (thumb..pinky).
    """
    full = list(curl_full_deg) if curl_full_deg is not None \
        else list(DEFAULT_CURL_FULL_DEG)
    if len(full) != 5:
        raise ValueError(f"curl_full_deg must have 5 entries, got {full!r}")
    e = _strip_side(ergo)
    drives: Dict[str, float] = {}
    for i, (short, long) in enumerate(_FINGERS):
        denom = max(1e-3, float(full[i]))
        s = _finger_stretch_deg(e, long)
        drives[f"{short}_curl"] = max(0.0, min(1.0, s / denom))
    drives["hand_curl"] = sum(
        drives[f"{short}_curl"] for short, _ in _FINGERS
    ) / len(_FINGERS)
    # Also expose raw per-joint stretches in [0,1] (some grasp modes
    # may want to map a single joint instead of the whole finger).
    for _short, long in _FINGERS:
        for jname in ("MCPStretch", "PIPStretch", "DIPStretch"):
            v = float(e.get(f"{long}{jname}", 0.0))
            # normalize each by 90 deg by default — matches typical
            # per-joint range. Still clamped to [0,1].
            drives[f"{long.lower()}_{jname.lower()[:3]}_stretch"] = \
                max(0.0, min(1.0, v / 90.0))
    return drives
