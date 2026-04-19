"""Reusable ManusGlove message builder.

Shared between the Qt slider UI and the headless e2e test so that both
paths construct identical messages.
"""

from typing import Dict

from manus_ros2_msgs.msg import ManusGlove, ManusErgonomics

ERGO_KEYS = [
    "ThumbMCPSpread",  "ThumbMCPStretch", "ThumbPIPStretch", "ThumbDIPStretch",
    "IndexSpread",     "IndexMCPStretch", "IndexPIPStretch", "IndexDIPStretch",
    "MiddleSpread",    "MiddleMCPStretch","MiddlePIPStretch","MiddleDIPStretch",
    "RingSpread",      "RingMCPStretch",  "RingPIPStretch",  "RingDIPStretch",
    "PinkySpread",     "PinkyMCPStretch", "PinkyPIPStretch", "PinkyDIPStretch",
]


def build_manus_glove_msg(
    values_deg: Dict[str, float],
    side: str = "Right",
    glove_id: int = 1,
) -> ManusGlove:
    """Populate a ManusGlove message with the 20 ergonomics fields.

    Missing keys default to 0.0. Extra keys are ignored.
    """
    msg = ManusGlove()
    msg.glove_id = int(glove_id)
    msg.side = side
    ergo_list = []
    for key in ERGO_KEYS:
        e = ManusErgonomics()
        e.type = key
        e.value = float(values_deg.get(key, 0.0))
        ergo_list.append(e)
    msg.ergonomics = ergo_list
    msg.ergonomics_count = len(ergo_list)
    msg.raw_node_count = 0
    msg.raw_nodes = []
    return msg
