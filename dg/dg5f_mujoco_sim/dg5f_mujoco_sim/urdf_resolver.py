"""Resolve dg5f_description URDF for MuJoCo.

MuJoCo's URDF compiler does not handle:
  - `package://` URIs   -> we substitute absolute file paths.
  - `.dae` mesh files   -> we redirect visual meshes to the matching
                            collision `.STL`. The collision STL set is
                            visually close enough for capture/inspection.
  - relative mesh paths -> we set `strippath="false"` via an injected
                            <mujoco><compiler/></mujoco> directive so
                            absolute paths are preserved.

Output URDF is cached in /tmp/dg5f_<side>_mujoco.urdf.
"""
from __future__ import annotations

import re
from pathlib import Path

DG5F_DESCRIPTION_PKG = "dg5f_description"


def _find_dg5f_description() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(DG5F_DESCRIPTION_PKG))
    except Exception:
        pass
    fallback = Path.home() / "hand_ws/src/dg/dg5f_ros2/dg5f_description"
    if fallback.exists():
        return fallback
    raise RuntimeError(
        f"could not locate '{DG5F_DESCRIPTION_PKG}' (tried ament index "
        f"and {fallback})"
    )


def resolve_dg5f_urdf(side: str) -> Path:
    if side not in ("left", "right"):
        raise ValueError(f"side must be left|right, got {side!r}")
    pkg = _find_dg5f_description()
    src_urdf = pkg / "urdf" / f"dg5f_{side}.urdf"
    if not src_urdf.exists():
        raise FileNotFoundError(src_urdf)

    text = src_urdf.read_text()
    meshes_root = (pkg / "meshes").absolute()
    text = text.replace(
        "package://dg5f_description/meshes/",
        str(meshes_root) + "/",
    )
    # MuJoCo can't load .dae natively here; swap visual .dae -> collision .STL.
    text = re.sub(
        r"/visual/([A-Za-z0-9_]+)\.dae",
        r"/collision/\1_c.STL",
        text,
    )
    inject = (
        '  <mujoco>\n'
        '    <compiler strippath="false" discardvisual="false" '
        'balanceinertia="true"/>\n'
        '  </mujoco>\n'
    )
    text = text.replace("</robot>", inject + "</robot>")

    out = Path("/tmp") / f"dg5f_{side}_mujoco.urdf"
    out.write_text(text)
    return out
