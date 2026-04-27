"""Pure-Python URDF parser + FK/Jacobian for DG5F-like serial finger chains.

Parses `<joint>` elements with translational `origin` (all rpy=0 in DG5F URDF)
and `axis` + `limit`. Builds a per-finger chain (4 revolute joints + 1 fixed
tip) and provides `fk`, `jacobian_tip_pos`, `jacobian_tip_dir`.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


def _vec(s: str) -> np.ndarray:
    return np.array([float(x) for x in s.split()], dtype=float)


@dataclass
class UrdfJoint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    axis: np.ndarray
    limit_lo: float
    limit_hi: float


def parse_urdf(urdf_path: str) -> Dict[str, UrdfJoint]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints: Dict[str, UrdfJoint] = {}
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        parent_el = j.find("parent")
        child_el = j.find("child")
        parent = parent_el.get("link") if parent_el is not None else ""
        child = child_el.get("link") if child_el is not None else ""
        origin = j.find("origin")
        xyz = np.zeros(3)
        if origin is not None and origin.get("xyz"):
            xyz = _vec(origin.get("xyz"))
        axis_el = j.find("axis")
        axis = np.array([0.0, 0.0, 1.0])
        if axis_el is not None and axis_el.get("xyz"):
            axis = _vec(axis_el.get("xyz"))
            n = np.linalg.norm(axis)
            axis = axis / (n if n > 0 else 1.0)
        lim = j.find("limit")
        lo = float(lim.get("lower")) if lim is not None and lim.get("lower") is not None else -np.pi
        hi = float(lim.get("upper")) if lim is not None and lim.get("upper") is not None else np.pi
        joints[name] = UrdfJoint(name, jtype, parent, child, xyz, axis, lo, hi)
    return joints


def _rodrigues(axis: np.ndarray, theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    x, y, z = axis
    C = 1.0 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


@dataclass
class FingerChain:
    origins: np.ndarray        # (4, 3) translations of each revolute joint (in parent frame)
    axes: np.ndarray           # (4, 3) unit axes
    tip_offset: np.ndarray     # (3,)   fixed translation from joint 4 child link to tip
    limits_lo: np.ndarray      # (4,)
    limits_hi: np.ndarray      # (4,)
    base_in_palm: np.ndarray   # (3,)   finger's joint-1 origin in palm frame
    joint_names: List[str]

    @property
    def n(self) -> int:
        return 4


def build_finger_chain(joints: Dict[str, UrdfJoint],
                       joint_names_4: List[str],
                       tip_joint_name: str,
                       palm_link: str) -> FingerChain:
    """Chain from `palm_link` to tip. First joint must be child of `palm_link`.

    We collect the 4 revolute joint origins as translations in the PARENT link
    frame (which, because rpy=0 in DG5F URDF, are just offsets). The tip fixed
    joint origin is the offset from joint-4's child link to the tip.
    """
    if joints[joint_names_4[0]].parent != palm_link:
        raise ValueError(
            f"first finger joint {joint_names_4[0]!r} parent is "
            f"{joints[joint_names_4[0]].parent!r}, expected {palm_link!r}"
        )
    origins = np.stack([joints[n].xyz for n in joint_names_4])
    axes = np.stack([joints[n].axis for n in joint_names_4])
    limits_lo = np.array([joints[n].limit_lo for n in joint_names_4])
    limits_hi = np.array([joints[n].limit_hi for n in joint_names_4])
    tip = joints[tip_joint_name].xyz
    return FingerChain(
        origins=origins, axes=axes, tip_offset=tip,
        limits_lo=limits_lo, limits_hi=limits_hi,
        base_in_palm=origins[0].copy(),
        joint_names=list(joint_names_4),
    )


def fk_finger(chain: FingerChain, q: np.ndarray) -> Dict[str, np.ndarray]:
    """Forward kinematics of one 4-DoF finger.

    Returns dict with:
      joint_pos (4, 3) — world position AFTER each origin translation, BEFORE
          that joint's rotation; that's the location of the rotation axis.
      joint_R   (4, 3, 3) — cumulative rotation BEFORE joint i (i.e. R_{0..i-1}).
      tip_pos   (3,)   — fingertip position (after all rotations + tip_offset).
      tip_dir   (3,)   — unit vector from joint-4 axis origin to tip.
      R_end     (3, 3) — cumulative rotation AFTER joint 4.
    """
    R = np.eye(3)
    p = np.zeros(3)
    joint_pos = np.zeros((4, 3))
    joint_R = np.zeros((4, 3, 3))
    for i in range(4):
        p = p + R @ chain.origins[i]
        joint_pos[i] = p
        joint_R[i] = R
        R = R @ _rodrigues(chain.axes[i], float(q[i]))
    tip_pos = p + R @ chain.tip_offset
    tip_vec = R @ chain.tip_offset
    nt = np.linalg.norm(tip_vec)
    tip_dir = tip_vec / (nt if nt > 0 else 1.0)
    return {
        "joint_pos": joint_pos,
        "joint_R": joint_R,
        "tip_pos": tip_pos,
        "tip_dir": tip_dir,
        "R_end": R,
    }


def jacobian_tip_pos(chain: FingerChain, q: np.ndarray,
                     fk: Dict[str, np.ndarray] | None = None) -> np.ndarray:
    """(3, 4) d tip_pos / d q."""
    if fk is None:
        fk = fk_finger(chain, q)
    tip = fk["tip_pos"]
    jp = fk["joint_pos"]
    jR = fk["joint_R"]
    J = np.zeros((3, 4))
    for i in range(4):
        axis_world = jR[i] @ chain.axes[i]
        J[:, i] = np.cross(axis_world, tip - jp[i])
    return J


def jacobian_tip_dir(chain: FingerChain, q: np.ndarray,
                     fk: Dict[str, np.ndarray] | None = None) -> np.ndarray:
    """(3, 4) d tip_dir / d q. d(R_end @ v_0)/dq_i = axis_world_i × (R_end @ v_0),
    where axis_world_i = R_{0..i-1} @ axes[i]. R_{0..i-1} is exactly
    fk["joint_R"][i] already cached by fk_finger — reuse it instead of
    recomputing rodrigues in a loop."""
    if fk is None:
        fk = fk_finger(chain, q)
    offset_n = chain.tip_offset / (np.linalg.norm(chain.tip_offset) or 1.0)
    end_offset = fk["R_end"] @ offset_n  # world tip-direction (unit)
    J = np.zeros((3, 4))
    jR = fk["joint_R"]
    for i in range(4):
        axis_world = jR[i] @ chain.axes[i]
        J[:, i] = np.cross(axis_world, end_offset)
    return J
