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


class SimState:
    """Sim state + dedicated render thread.

    EGL contexts are tied to the thread that created them, so all
    `mujoco.Renderer` calls must happen on a single thread. The render
    thread owns the model/data/renderer; HTTP handlers only push pose
    requests via `pending_q` and read the latest JPEG out of `latest_jpeg`.
    """

    def __init__(self, side: str, width: int, height: int, fps: int):
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

        renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.lookat[:] = [0.0, 0.0, 0.05]
        cam.distance = 0.30
        cam.elevation = -20.0
        cam.azimuth = 90.0 if self.side == "left" else -90.0

        mujoco.mj_forward(model, data)
        period = 1.0 / self.fps

        try:
            while not self._stop.is_set():
                t0 = time.monotonic()

                self._poll_pose_file()
                with self.lock:
                    pending = self.pending_q
                    label = self.pending_label
                    self.pending_q = None
                if pending is not None:
                    for i, qi in enumerate(qpos_idx):
                        data.qpos[qi] = float(pending[i])
                    mujoco.mj_forward(model, data)
                    with self.lock:
                        self.target_q = pending.copy()
                        if label:
                            self.pose_label = label

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
</style></head>
<body>
<h2>DG5F MuJoCo — {side} hand</h2>
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
            )

    return app


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--side", choices=["left", "right"], default="left")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    args = p.parse_args(argv)

    state = SimState(args.side, args.width, args.height, fps=args.fps)
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
