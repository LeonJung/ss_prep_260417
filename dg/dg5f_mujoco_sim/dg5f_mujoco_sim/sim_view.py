"""MuJoCo kinematic sim of DG5F + Flask MJPEG viewer.

Run:
  python3 -m dg5f_mujoco_sim.sim_view --side left
  ros2 run dg5f_mujoco_sim sim_view --ros-args -p side:=left   # not used; CLI only

Then in another terminal:
  cloudflared tunnel --url http://localhost:8080

Pose control:
  - HTTP POST /pose with JSON {"side":"left","q":[...20 floats in rad...]}
  - or write the same JSON to /tmp/dg5f_sim_pose.json — the sim picks up
    file-mtime changes every render tick.

The 20 q-vector slots match the manus_dg5f_retarget convention:
  0..3  thumb  (lj_dg_1_1..1_4)
  4..7  index  (lj_dg_2_*)
  8..11 middle (lj_dg_3_*)
  12..15 ring  (lj_dg_4_*)
  16..19 pinky (lj_dg_5_*)
Right hand uses rj_*. The sim only renders the side it was launched for.
"""
from __future__ import annotations

import argparse
import io
import json
import os
import threading
import time
from pathlib import Path
from typing import List

# Pick GL backend before importing mujoco. EGL works headless on Linux
# with a GPU; if that fails users can `export MUJOCO_GL=osmesa`.
os.environ.setdefault("MUJOCO_GL", "egl")

import flask
import mujoco
import numpy as np
from PIL import Image

from .urdf_resolver import resolve_dg5f_urdf

POSE_FILE = "/tmp/dg5f_sim_pose.json"

JOINT_ORDER_LEFT: List[str] = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]
JOINT_ORDER_RIGHT: List[str] = [n.replace("lj_", "rj_") for n in JOINT_ORDER_LEFT]


# MuJoCo merges fixed-joint bodies on URDF import — so the *_tip and *_palm
# links don't appear as separate bodies. We resolve tip frames by hand: each
# tip is at a fixed offset from the j_4 body (URDF lj_dg_<f>_tip joint
# origin), and palm is a fixed offset from world along +Z (lj_dg_base +
# lj_dg_palm origins, both rotation-free).
#
# Values copied from dg5f_description/urdf/dg5f_{left,right}.urdf.
TIP_OFFSETS_LEFT = [
    (0.0, -0.0363, 0.0),   # thumb
    (0.0,  0.0,    0.0255),  # index
    (0.0,  0.0,    0.0255),  # middle
    (0.0,  0.0,    0.0255),  # ring
    (0.0,  0.0,    0.0363),  # pinky
]
TIP_OFFSETS_RIGHT = [
    (0.0,  0.0363, 0.0),   # thumb (mirrored Y)
    (0.0,  0.0,    0.0255),
    (0.0,  0.0,    0.0255),
    (0.0,  0.0,    0.0255),
    (0.0,  0.0,    0.0363),
]
PALM_WORLD_OFFSET = (0.0, 0.0, 0.0738)  # mount->base + base->palm


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, (w,x,y,z) convention."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


class SimState:
    """Sim state + dedicated render thread.

    EGL contexts are tied to the thread that created them, so all
    `mujoco.Renderer` calls must happen on a single thread. The render
    thread owns the model/data/renderer; HTTP handlers only push pose
    requests via `pending_q` and read the latest JPEG out of `latest_jpeg`.
    """

    def __init__(self, side: str, width: int, height: int, fps: int,
                 cam_distance: float, cam_azimuth: float, cam_elevation: float,
                 cam_lookat: tuple):
        self.side = side
        self.width = width
        self.height = height
        self.fps = max(1, fps)
        self._urdf_path = resolve_dg5f_urdf(side)

        self.lock = threading.Lock()
        self.target_q = np.zeros(20, dtype=np.float64)
        self.pending_q: np.ndarray | None = None
        self.pending_label: str = ""
        self.last_pose_mtime = 0.0
        self.pose_label = "open"
        self.latest_jpeg: bytes = b""
        # Camera state — held under self.lock; the render thread reads
        # these every tick so HTTP /camera POST takes effect immediately.
        self.cam_distance = float(cam_distance)
        self.cam_azimuth = float(cam_azimuth)
        self.cam_elevation = float(cam_elevation)
        self.cam_lookat = tuple(float(x) for x in cam_lookat)
        # Cached tip poses (palm-frame) — refreshed after each mj_forward.
        # Each entry: dict(name, pos=[x,y,z], quat=[w,x,y,z]).
        self.tip_poses: List[dict] = []
        self._stop = threading.Event()
        self._frame_event = threading.Event()
        self._thread = threading.Thread(target=self._render_loop, daemon=True)

    def start(self):
        self._thread.start()
        # Wait briefly for first frame so the HTTP handler doesn't 500.
        for _ in range(50):
            if self.latest_jpeg:
                return
            time.sleep(0.05)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def request_q(self, q: np.ndarray, label: str = ""):
        with self.lock:
            self.pending_q = np.asarray(q, dtype=np.float64).copy()
            self.pending_label = label

    def update_camera(self, distance=None, azimuth=None, elevation=None,
                      lookat=None):
        with self.lock:
            if distance is not None:
                self.cam_distance = float(distance)
            if azimuth is not None:
                self.cam_azimuth = float(azimuth)
            if elevation is not None:
                self.cam_elevation = float(elevation)
            if lookat is not None:
                self.cam_lookat = tuple(float(x) for x in lookat)

    def _poll_pose_file(self) -> None:
        try:
            mtime = os.path.getmtime(POSE_FILE)
        except OSError:
            return
        if mtime <= self.last_pose_mtime:
            return
        try:
            with open(POSE_FILE) as f:
                d = json.load(f)
        except (OSError, json.JSONDecodeError):
            return
        if d.get("side") != self.side:
            return
        q = d.get("q")
        if not isinstance(q, list) or len(q) != 20:
            return
        self.request_q(np.asarray(q, dtype=np.float64),
                       label=str(d.get("label", "")))
        self.last_pose_mtime = mtime

    def wait_for_frame(self, timeout: float = 1.0) -> None:
        self._frame_event.wait(timeout=timeout)
        self._frame_event.clear()

    def _compute_tip_poses(self, model, data, j4_bids: List[int],
                           tip_offsets: List[tuple],
                           finger_names: List[str]) -> List[dict]:
        """Tip poses in the palm frame.

        Each tip is at a fixed offset (URDF lj_dg_<f>_tip joint origin)
        from the j_4 body it lives on. Palm has no rotation w.r.t. world,
        so palm-frame pose = world-frame pose minus PALM_WORLD_OFFSET.
        """
        palm_pos = np.asarray(PALM_WORLD_OFFSET, dtype=np.float64)
        out = []
        for name, j4_bid, offset in zip(finger_names, j4_bids, tip_offsets):
            j4_pos = data.xpos[j4_bid]
            j4_mat = data.xmat[j4_bid].reshape(3, 3)
            j4_q = data.xquat[j4_bid]
            tip_world = j4_pos + j4_mat @ np.asarray(offset, dtype=np.float64)
            pos_palm = tip_world - palm_pos
            out.append(dict(
                name=name,
                pos=[float(x) for x in pos_palm],
                quat=[float(x) for x in j4_q],
            ))
        return out

    def _render_loop(self):
        # All mujoco/EGL state stays on this thread.
        model = mujoco.MjModel.from_xml_path(str(self._urdf_path))
        data = mujoco.MjData(model)
        order = JOINT_ORDER_LEFT if self.side == "left" else JOINT_ORDER_RIGHT
        qpos_idx: List[int] = []
        for name in order:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"joint not found in URDF: {name}")
            qpos_idx.append(int(model.jnt_qposadr[jid]))

        prefix = "ll_" if self.side == "left" else "rl_"
        finger_names = ["thumb", "index", "middle", "ring", "pinky"]
        # MuJoCo merges fixed-joint *_tip links into the parent j_4 body.
        # We anchor on j_4 and add the URDF tip-joint offset by hand.
        j4_bids: List[int] = []
        for f in range(1, 6):
            j4_name = f"{prefix}dg_{f}_4"
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, j4_name)
            if bid < 0:
                raise RuntimeError(f"j_4 body not found: {j4_name}")
            j4_bids.append(bid)
        tip_offsets = (TIP_OFFSETS_LEFT if self.side == "left"
                       else TIP_OFFSETS_RIGHT)

        renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)

        mujoco.mj_forward(model, data)
        # Initial tip poses
        with self.lock:
            self.tip_poses = self._compute_tip_poses(
                model, data, j4_bids, tip_offsets, finger_names)
        period = 1.0 / self.fps

        try:
            while not self._stop.is_set():
                t0 = time.monotonic()

                self._poll_pose_file()
                with self.lock:
                    pending = self.pending_q
                    label = self.pending_label
                    self.pending_q = None
                    cam.lookat[:] = list(self.cam_lookat)
                    cam.distance = self.cam_distance
                    cam.elevation = self.cam_elevation
                    cam.azimuth = self.cam_azimuth
                if pending is not None:
                    for i, qi in enumerate(qpos_idx):
                        data.qpos[qi] = float(pending[i])
                    mujoco.mj_forward(model, data)
                    tips = self._compute_tip_poses(
                        model, data, j4_bids, tip_offsets, finger_names)
                    with self.lock:
                        self.target_q = pending.copy()
                        if label:
                            self.pose_label = label
                        self.tip_poses = tips

                renderer.update_scene(data, camera=cam)
                arr = renderer.render()
                img = Image.fromarray(arr)
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=80)
                with self.lock:
                    self.latest_jpeg = buf.getvalue()
                self._frame_event.set()

                dt = time.monotonic() - t0
                sleep_s = period - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)
        finally:
            try:
                renderer.close()
            except Exception:
                pass


_HTML = """<!doctype html>
<html><head><title>DG5F sim — {side}</title>
<style>
  body {{ background:#222; color:#ddd; font-family:monospace;
         text-align:center; margin:0; padding:1em; }}
  img  {{ border:1px solid #444; max-width:95vw; height:auto; }}
  code {{ color:#9cf; }}
  .hint {{ color:#888; font-size:90%; }}
  .nav  {{ font-size:90%; margin-bottom:0.5em; }}
</style></head>
<body>
<h2>DG5F MuJoCo — {side} hand</h2>
<div class="nav">
  view:
  <a href="/" style="color:#9cf;">stream (MJPEG)</a> ·
  <a href="/poll" style="color:#9cf;">poll (snapshot)</a>
</div>
<img src="/stream"/>
<p class="hint">Pose control:
  <br>POST <code>/pose</code> with <code>{{"side":"{side}","q":[...20 rad...],
  "label":"open"}}</code>
  <br>or write same JSON to <code>{pose_file}</code>.
</p>
<p class="hint">Status: <span id="lbl">…</span></p>
<script>
async function poll() {{
  try {{
    const r = await fetch('/status');
    const d = await r.json();
    document.getElementById('lbl').textContent = d.label || '(unset)';
  }} catch(e) {{}}
  setTimeout(poll, 500);
}}
poll();
</script>
</body></html>
"""

# Polling fallback for clients that don't render multipart/x-mixed-replace
# (some mobile browsers and corporate proxies). Each cycle fetches one
# JPEG via /snapshot and swaps it in.
_HTML_POLL = """<!doctype html>
<html><head><title>DG5F sim — {side} (poll)</title>
<style>
  body {{ background:#222; color:#ddd; font-family:monospace;
         text-align:center; margin:0; padding:1em; }}
  img  {{ border:1px solid #444; max-width:95vw; height:auto; }}
  .nav {{ font-size:90%; margin-bottom:0.5em; }}
  .hint {{ color:#888; font-size:90%; }}
</style></head>
<body>
<h2>DG5F MuJoCo — {side} hand (poll mode)</h2>
<div class="nav">
  view:
  <a href="/" style="color:#9cf;">stream (MJPEG)</a> ·
  <a href="/poll" style="color:#9cf;">poll (snapshot)</a>
</div>
<img id="frame"/>
<p class="hint">Status: <span id="lbl">…</span></p>
<script>
const img = document.getElementById('frame');
function tick() {{
  // Cache-bust by appending a counter so the browser refetches.
  img.src = '/snapshot?t=' + Date.now();
}}
img.addEventListener('load',  () => setTimeout(tick, 50));
img.addEventListener('error', () => setTimeout(tick, 500));
tick();
async function poll() {{
  try {{
    const r = await fetch('/status');
    const d = await r.json();
    document.getElementById('lbl').textContent = d.label || '(unset)';
  }} catch(e) {{}}
  setTimeout(poll, 500);
}}
poll();
</script>
</body></html>
"""


def make_app(state: SimState) -> flask.Flask:
    app = flask.Flask(__name__)

    @app.get("/")
    def index():
        return _HTML.format(side=state.side, pose_file=POSE_FILE)

    def gen():
        while True:
            state.wait_for_frame(timeout=1.0)
            with state.lock:
                jpeg = state.latest_jpeg
            if not jpeg:
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")

    @app.get("/stream")
    def stream():
        return flask.Response(
            gen(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/poll")
    def poll_page():
        return _HTML_POLL.format(side=state.side)

    @app.get("/snapshot")
    def snapshot():
        # Wait briefly so we serve a fresh frame after recent pose updates.
        state.wait_for_frame(timeout=0.5)
        with state.lock:
            jpeg = state.latest_jpeg
        if not jpeg:
            return ("no frame yet", 503)
        resp = flask.Response(jpeg, mimetype="image/jpeg")
        resp.headers["Cache-Control"] = "no-store"
        return resp

    @app.post("/pose")
    def post_pose():
        d = flask.request.get_json(force=True, silent=True) or {}
        if d.get("side") != state.side:
            return flask.jsonify(ok=False, err="side mismatch"), 400
        q = d.get("q")
        if not isinstance(q, list) or len(q) != 20:
            return flask.jsonify(ok=False, err="q must be 20-list"), 400
        try:
            arr = np.asarray(q, dtype=np.float64)
        except Exception as e:
            return flask.jsonify(ok=False, err=str(e)), 400
        state.request_q(arr, label=str(d.get("label", "")))
        return flask.jsonify(ok=True, label=state.pose_label)

    @app.get("/status")
    def status():
        with state.lock:
            return flask.jsonify(
                side=state.side,
                label=state.pose_label,
                q=state.target_q.tolist(),
                cam=dict(
                    distance=state.cam_distance,
                    azimuth=state.cam_azimuth,
                    elevation=state.cam_elevation,
                    lookat=list(state.cam_lookat),
                ),
            )

    @app.get("/tips")
    def tips():
        with state.lock:
            return flask.jsonify(
                side=state.side,
                frame="palm",
                tips=list(state.tip_poses),
            )

    @app.post("/camera")
    def post_camera():
        d = flask.request.get_json(force=True, silent=True) or {}
        try:
            state.update_camera(
                distance=d.get("distance"),
                azimuth=d.get("azimuth"),
                elevation=d.get("elevation"),
                lookat=d.get("lookat"),
            )
        except Exception as e:
            return flask.jsonify(ok=False, err=str(e)), 400
        return flask.jsonify(ok=True)

    return app


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--side", choices=["left", "right"], default="left")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    # Camera defaults: framed to show the whole hand from a 45° angle
    # off the palm-normal. Tweak via /camera POST or these flags.
    p.add_argument("--cam-distance", type=float, default=0.55)
    p.add_argument("--cam-azimuth", type=float, default=None,
                   help="default: 45 (left) / -45 (right)")
    p.add_argument("--cam-elevation", type=float, default=-15.0)
    p.add_argument("--cam-lookat", type=str, default="0,0,0.10",
                   help="x,y,z comma-separated")
    args = p.parse_args(argv)

    azimuth = (args.cam_azimuth if args.cam_azimuth is not None
               else (45.0 if args.side == "left" else -45.0))
    lookat = tuple(float(x) for x in args.cam_lookat.split(","))
    if len(lookat) != 3:
        raise ValueError(f"--cam-lookat must be 3 numbers, got {args.cam_lookat!r}")

    state = SimState(args.side, args.width, args.height, fps=args.fps,
                     cam_distance=args.cam_distance,
                     cam_azimuth=azimuth,
                     cam_elevation=args.cam_elevation,
                     cam_lookat=lookat)
    state.start()
    app = make_app(state)
    print(f"[dg5f_mujoco_sim] side={args.side} "
          f"http://{args.host}:{args.port}/  pose_file={POSE_FILE}",
          flush=True)
    try:
        app.run(host=args.host, port=args.port, threaded=True, debug=False)
    finally:
        state.stop()


if __name__ == "__main__":
    main()
