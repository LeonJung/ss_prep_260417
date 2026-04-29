"""Interactive grasp-mode capture CLI.

Talks to a running sim_view (HTTP localhost:8080 by default). Lets you:
  - apply a 20-vec or per-finger 4-vec pose
  - inspect the current pose + fingertip positions (palm frame)
  - save the current pose as a named grasp mode in
    config/grasp_modes_<side>.yaml (alongside fingertip poses + description)
  - load a saved grasp mode back into the sim
  - list / remove saved modes

Slot ordering (matches manus_dg5f_retarget):
  0..3   thumb  (j_dg_1_*)
  4..7   index  (j_dg_2_*)
  8..11  middle (j_dg_3_*)
  12..15 ring   (j_dg_4_*)
  16..19 pinky  (j_dg_5_*)

Examples:
  capture_cli show
  capture_cli set --q "0,0.5,1.0,0.5,0,0.6,1.5,0.8,..."
  capture_cli set --finger thumb --q "0.0,0.5,1.0,0.5"
  capture_cli save key_grip --desc "side-pinch on index middle phalanx"
  capture_cli load key_grip
  capture_cli list
  capture_cli remove key_grip
  capture_cli reset
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import urllib.request
import urllib.error

import yaml

DEFAULT_BASE_URL = "http://127.0.0.1:8080"

FINGER_SLOT_RANGES = {
    "thumb":  (0, 4),
    "index":  (4, 8),
    "middle": (8, 12),
    "ring":   (12, 16),
    "pinky":  (16, 20),
}


def _http(method: str, url: str, payload: Optional[dict] = None) -> dict:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            body = resp.read().decode("utf-8")
    except urllib.error.URLError as e:
        raise SystemExit(f"sim not reachable at {url}: {e}")
    try:
        return json.loads(body) if body else {}
    except json.JSONDecodeError:
        raise SystemExit(f"non-json response from {url}: {body[:200]}")


def get_status(base: str) -> dict:
    return _http("GET", f"{base}/status")


def get_tips(base: str) -> dict:
    return _http("GET", f"{base}/tips")


def post_pose(base: str, q: List[float], label: str, side: str) -> dict:
    return _http("POST", f"{base}/pose",
                 dict(side=side, q=list(q), label=label))


def parse_q_csv(text: str) -> List[float]:
    parts = [p.strip() for p in text.replace(";", ",").split(",")]
    parts = [p for p in parts if p]
    return [float(p) for p in parts]


def yaml_path_for(side: str) -> Path:
    """Find config/grasp_modes_<side>.yaml; prefer the source-tree copy
    in the workspace so subsequent colcon build picks it up. Fall back
    to install/share location if source not found.
    """
    # 1) workspace source tree
    src = (Path.home() /
           f"hand_ws/src/dg/dg5f_mujoco_sim/config/grasp_modes_{side}.yaml")
    if src.exists() or src.parent.exists():
        return src
    # 2) installed share
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("dg5f_mujoco_sim"))
        return share / "config" / f"grasp_modes_{side}.yaml"
    except Exception:
        pass
    raise RuntimeError(f"could not determine grasp_modes_{side}.yaml location")


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return dict(side=path.stem.replace("grasp_modes_", ""), modes={})
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    if "modes" not in d:
        d["modes"] = {}
    return d


def save_yaml(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        f"# Auto-edited by dg5f_mujoco_sim/capture_cli at "
        f"{datetime.datetime.now().isoformat(timespec='seconds')}\n"
    )
    with open(path, "w") as f:
        f.write(header)
        yaml.safe_dump(payload, f, sort_keys=False, default_flow_style=False)


# ---------- subcommands ----------

def cmd_show(args):
    base = args.base
    st = get_status(base)
    tp = get_tips(base)
    side = st["side"]
    q = st["q"]
    print(f"side: {side}")
    print(f"label: {st['label']}")
    print("q (rad):")
    fingers = ["thumb", "index", "middle", "ring", "pinky"]
    for fi, fn in enumerate(fingers):
        seg = q[fi * 4:(fi + 1) * 4]
        print(f"  {fn:6s} [{fi*4:2d}..{fi*4+3:2d}]  "
              + "  ".join(f"{v:+.3f}" for v in seg))
    print("tip pose (palm frame):")
    for tip in tp.get("tips", []):
        pos = tip["pos"]
        print(f"  {tip['name']:6s}  pos=({pos[0]*1000:6.1f}, "
              f"{pos[1]*1000:6.1f}, {pos[2]*1000:6.1f}) mm")


def cmd_set(args):
    base = args.base
    st = get_status(base)
    side = st["side"]
    cur_q: List[float] = list(st["q"])
    vals = parse_q_csv(args.q)
    if args.finger:
        if args.finger not in FINGER_SLOT_RANGES:
            raise SystemExit(f"unknown finger {args.finger}; choose from "
                             f"{list(FINGER_SLOT_RANGES)}")
        lo, hi = FINGER_SLOT_RANGES[args.finger]
        if len(vals) != 4:
            raise SystemExit(f"--finger requires exactly 4 values, got {len(vals)}")
        cur_q[lo:hi] = vals
        new_label = f"set:{args.finger}"
    else:
        if len(vals) != 20:
            raise SystemExit(f"need 20 values for --q (no --finger), got {len(vals)}")
        cur_q = list(vals)
        new_label = "set:all"
    label = args.label or new_label
    resp = post_pose(base, cur_q, label, side)
    if not resp.get("ok"):
        raise SystemExit(f"sim refused pose: {resp}")
    print(f"applied  side={side}  label={label}")


def cmd_save(args):
    base = args.base
    st = get_status(base)
    side = st["side"]
    tp = get_tips(base)
    path = yaml_path_for(side)
    db = load_yaml(path)
    db["side"] = side
    db.setdefault("modes", {})
    if args.name in db["modes"] and not args.overwrite:
        raise SystemExit(f"mode {args.name!r} already exists; rerun with "
                         f"--overwrite to replace")
    db["modes"][args.name] = dict(
        description=args.desc,
        q=[float(v) for v in st["q"]],
        tip_pose={t["name"]: dict(pos=t["pos"], quat=t["quat"])
                  for t in tp.get("tips", [])},
        timestamp=datetime.datetime.now().isoformat(timespec="seconds"),
    )
    save_yaml(path, db)
    print(f"saved {args.name!r} -> {path}")


def cmd_load(args):
    base = args.base
    st = get_status(base)
    side = st["side"]
    path = yaml_path_for(side)
    db = load_yaml(path)
    modes = db.get("modes", {})
    if args.name not in modes:
        raise SystemExit(f"mode {args.name!r} not found in {path}; "
                         f"available: {list(modes)}")
    q = modes[args.name].get("q")
    if not isinstance(q, list) or len(q) != 20:
        raise SystemExit(f"bad q in {path}::modes::{args.name}")
    resp = post_pose(base, q, f"load:{args.name}", side)
    if not resp.get("ok"):
        raise SystemExit(f"sim refused pose: {resp}")
    print(f"loaded {args.name!r}  side={side}")


def cmd_list(args):
    side = args.side or get_status(args.base)["side"]
    path = yaml_path_for(side)
    db = load_yaml(path)
    modes = db.get("modes", {})
    if not modes:
        print(f"(no modes saved in {path})")
        return
    print(f"saved modes in {path}:")
    for name, m in modes.items():
        desc = m.get("description") or ""
        ts = m.get("timestamp") or ""
        print(f"  {name:18s}  {ts}  {desc}")


def cmd_remove(args):
    side = args.side or get_status(args.base)["side"]
    path = yaml_path_for(side)
    db = load_yaml(path)
    modes = db.get("modes", {})
    if args.name not in modes:
        raise SystemExit(f"mode {args.name!r} not in {path}")
    del modes[args.name]
    save_yaml(path, db)
    print(f"removed {args.name!r} from {path}")


def cmd_reset(args):
    base = args.base
    st = get_status(base)
    side = st["side"]
    resp = post_pose(base, [0.0] * 20, "reset", side)
    if not resp.get("ok"):
        raise SystemExit(f"sim refused pose: {resp}")
    print(f"reset all 20 slots to 0  side={side}")


def main(argv=None):
    p = argparse.ArgumentParser(prog="capture_cli")
    p.add_argument("--base", default=os.environ.get("DG5F_SIM_URL", DEFAULT_BASE_URL),
                   help="sim_view base URL (default: %(default)s)")
    sp = p.add_subparsers(dest="cmd", required=True)

    s = sp.add_parser("show", help="print current q + tip poses")
    s.set_defaults(func=cmd_show)

    s = sp.add_parser("set", help="apply a pose")
    s.add_argument("--q", required=True, help="comma-separated radians")
    s.add_argument("--finger", choices=list(FINGER_SLOT_RANGES),
                   help="restrict --q to one finger (4 values)")
    s.add_argument("--label", default=None)
    s.set_defaults(func=cmd_set)

    s = sp.add_parser("save", help="store current pose as a grasp mode")
    s.add_argument("name")
    s.add_argument("--desc", default="")
    s.add_argument("--overwrite", action="store_true")
    s.set_defaults(func=cmd_save)

    s = sp.add_parser("load", help="apply a saved grasp mode")
    s.add_argument("name")
    s.set_defaults(func=cmd_load)

    s = sp.add_parser("list", help="list saved grasp modes")
    s.add_argument("--side", choices=["left", "right"], default=None,
                   help="default: query running sim")
    s.set_defaults(func=cmd_list)

    s = sp.add_parser("remove", help="delete a saved grasp mode")
    s.add_argument("name")
    s.add_argument("--side", choices=["left", "right"], default=None)
    s.set_defaults(func=cmd_remove)

    s = sp.add_parser("reset", help="apply all-zeros (open hand)")
    s.set_defaults(func=cmd_reset)

    args = p.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
