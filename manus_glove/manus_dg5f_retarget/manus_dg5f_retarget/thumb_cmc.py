"""Thumb CMC (carpometacarpal / adductor pollicis) estimation.

Manus ergonomics does NOT expose the thumb CMC abduction axis directly. The
only thumb ergonomics fields published by the Manus ROS2 package are
ThumbMCPSpread / ThumbMCPStretch / ThumbPIPStretch / ThumbDIPStretch.
DG5F's rj_dg_1_1 joint represents the thumb-base abduction — we need to
supply it separately.

Three modes:
  * fixed        — hold at a constant value (default 0.0). Safest.
  * coupled      — linear blend of ThumbMCPSpread and ThumbMCPStretch.
                   This mirrors the heuristic from the original
                   manus_retarget.py formula:
                       qd[0] = (58.5 - ThumbMCPStretch) * deg2rad
                   We generalize it as:
                       out = offset_deg - gain_stretch * MCPStretch
                             + gain_spread * MCPSpread
                       (all in degrees, converted to rad at the end)
  * raw_nodes_ik — estimate from raw_nodes[] thumb chain. Stub only;
                   requires knowing which node_ids Manus assigns to the
                   thumb chain, which must be confirmed with a real
                   Manus session. Falls back to `fixed` until mapped.
"""

import math

DEG = math.pi / 180.0

# Defaults mirror the original manus_retarget.py heuristic for the right hand:
#   qd[0] = (58.5 - ThumbMCPStretch) * DEG   with sign = +1 for right hand
# Expressed in our coupled formula:
#   offset_deg = 58.5, gain_stretch = 1.0, gain_spread = 0.0
DEFAULT_COUPLED_OFFSET_DEG = 58.5
DEFAULT_COUPLED_GAIN_STRETCH = 1.0
DEFAULT_COUPLED_GAIN_SPREAD = 0.0


def compute_thumb_cmc(
    mode: str,
    mcp_spread_deg: float,
    mcp_stretch_deg: float,
    raw_nodes=None,
    fixed_value_rad: float = 0.0,
    offset_deg: float = DEFAULT_COUPLED_OFFSET_DEG,
    gain_stretch: float = DEFAULT_COUPLED_GAIN_STRETCH,
    gain_spread: float = DEFAULT_COUPLED_GAIN_SPREAD,
) -> float:
    """Return thumb CMC angle in radians (output joint space, pre-clamp).

    The caller is expected to apply per-joint sign / gain / limits on top.
    """
    if mode == "fixed":
        return float(fixed_value_rad)

    if mode == "coupled":
        deg = offset_deg - gain_stretch * mcp_stretch_deg + gain_spread * mcp_spread_deg
        return deg * DEG

    if mode == "raw_nodes_ik":
        # TODO: decode raw_nodes[] thumb chain:
        #   1) find thumb CMC node and MCP node by chain_type / joint_type
        #   2) express MCP pose in CMC-local frame
        #   3) extract the abduction angle about the CMC abduction axis
        # Until the Manus skeleton node_ids are confirmed with a live glove,
        # this path falls back to `fixed` and emits a warning upstream.
        return float(fixed_value_rad)

    # Unknown mode — safest fallback.
    return float(fixed_value_rad)
