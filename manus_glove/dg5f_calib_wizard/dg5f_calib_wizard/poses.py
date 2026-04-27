"""Canonical hand poses the wizard walks the user through."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    name: str
    instruction: str
    finger_index: int = -1   # -1 = N/A, else 1..4 (index/middle/ring/pinky)
    is_pad: bool = False


# Order matters — wizard runs them in this sequence.
POSES = [
    Pose(
        name="open",
        instruction=(
            "OPEN HAND — Spread your fingers as if showing your palm.\n"
            "  All five fingers fully extended, thumb relaxed away from palm."
        ),
    ),
    Pose(
        name="fist",
        instruction=(
            "FIST — Curl all five fingers into a tight fist.\n"
            "  Thumb folds over the index/middle middle phalanges."
        ),
    ),
    Pose(
        name="tip_index",
        instruction=(
            "TIP PINCH (Index) — Touch your thumbnail to your index nail.\n"
            "  Both fingertips meet pad-to-pad, like picking up a small bead."
        ),
        finger_index=1,
    ),
    Pose(
        name="tip_middle",
        instruction=(
            "TIP PINCH (Middle) — Touch your thumbnail to your middle finger nail."
        ),
        finger_index=2,
    ),
    Pose(
        name="tip_ring",
        instruction=(
            "TIP PINCH (Ring) — Touch your thumbnail to your ring finger nail."
        ),
        finger_index=3,
    ),
    Pose(
        name="tip_pinky",
        instruction=(
            "TIP PINCH (Pinky) — Touch your thumbnail to your pinky nail."
        ),
        finger_index=4,
    ),
    Pose(
        name="pad_index",
        instruction=(
            "PAD PINCH (Index) — Hold an imaginary key.\n"
            "  Thumb pad pressed against the SIDE of your index middle phalanx,\n"
            "  index DIP slightly extended (almost straight)."
        ),
        finger_index=1,
        is_pad=True,
    ),
    Pose(
        name="pad_middle",
        instruction=(
            "PAD PINCH (Middle) — Same as the index pad pinch but with middle finger."
        ),
        finger_index=2,
        is_pad=True,
    ),
]


POSE_BY_NAME = {p.name: p for p in POSES}
