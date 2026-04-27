"""Manus ergonomics -> DG5F pid_all reference via SLSQP optimization (A-base).

Compared to manus_dg5f_retarget.retarget_node (ergo-only warm start publish),
this node adds a kinematic optimization step with a comprehensive retargeting
objective (prior, velocity, fingertip direction, sigmoid-switched pinch)
solved against the DG5F URDF. The ergo-mapped solution is used as the warm
start + prior, so behavior degrades gracefully to the old mapping when pinch
intent is absent.

Publishes control_msgs/MultiDOFCommand on /dg5f_{right,left}/{rj,lj}_dg_pospid
/reference just like the original node. Supports optional `contact_aware`
modulation using /dg5f_<side>/contact_level + joint_states.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from control_msgs.msg import MultiDOFCommand
from manus_ros2_msgs.msg import ManusGlove
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from .contact_modulation import ContactConfig, modulate
from .ergo_map import (
    CALIB_DEFAULT, DIR_LEFT, DIR_RIGHT, ERGO_KEYS,
    POSTPROC_LEFT, POSTPROC_RIGHT,
    compute_thumb_cmc, ergo_to_q0_rad,
)
from .retarget_ik import (
    AObjectiveConfig, AObjectiveWeights, solve_ik,
)
from .urdf_fk import build_finger_chain, parse_urdf


RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]
LEFT_JOINT_NAMES = [n.replace("rj_", "lj_") for n in RIGHT_JOINT_NAMES]


def _default_urdf_path(side: str) -> str:
    share = get_package_share_directory("dg5f_description")
    name = "dg5f_right.urdf" if side == "right" else "dg5f_left.urdf"
    return os.path.join(share, "urdf", name)


def _build_cfg(side: str, urdf_path: str, weights: AObjectiveWeights) -> AObjectiveConfig:
    joints = parse_urdf(urdf_path)
    prefix = "rj" if side == "right" else "lj"
    palm_link = f"{prefix[0]}l_dg_palm"
    chains = []
    for f in range(1, 6):
        jn = [f"{prefix}_dg_{f}_{k}" for k in range(1, 5)]
        tip = f"{prefix}_dg_{f}_tip"
        chains.append(build_finger_chain(joints, jn, tip, palm_link))
    bounds_lo = np.concatenate([ch.limits_lo for ch in chains])
    bounds_hi = np.concatenate([ch.limits_hi for ch in chains])
    return AObjectiveConfig(
        weights=weights,
        finger_chains=chains,
        bounds_lo=bounds_lo,
        bounds_hi=bounds_hi,
        max_iter=25,
        ftol=1e-5,
    )


class ManusDg5fSotaRetargetA(Node):
    def __init__(self):
        super().__init__("manus_dg5f_sota_retargeting_a_good")

        # Side + topics
        self.declare_parameter("hand_side", "right")
        self.declare_parameter("input_topic", "/manus_glove_1")
        self.declare_parameter("output_topic", "/dg5f_right/rj_dg_pospid/reference")
        self.declare_parameter("expected_side", "right")
        self.declare_parameter("joint_states_topic", "/dg5f_right/joint_states")
        self.declare_parameter("contact_level_topic", "/dg5f_right/contact_level")

        # Warm-start pipeline knobs (mirror manus_dg5f_retarget)
        self.declare_parameter("thumb_cmc_mode", "fixed")    # fixed | coupled
        self.declare_parameter("thumb_cmc_fixed_value_rad", 0.0)
        self.declare_parameter("thumb_cmc_offset_deg", 58.5)
        self.declare_parameter("thumb_cmc_gain_stretch", 1.0)
        self.declare_parameter("thumb_cmc_gain_spread", 0.0)
        self.declare_parameter("calib", CALIB_DEFAULT)
        self.declare_parameter("dir_sign", [float(s) for s in DIR_RIGHT.tolist()])

        # URDF
        self.declare_parameter("urdf_path", "")

        # Objective weights
        self.declare_parameter("w_prior", 1.0)
        self.declare_parameter("w_velocity", 0.2)
        self.declare_parameter("w_orient", 0.3)
        self.declare_parameter("w_pinch", 4.0)
        self.declare_parameter("pinch_close_ref_m", 0.030)
        self.declare_parameter("pinch_far_ref_m", 0.120)
        self.declare_parameter("pinch_target_min_m", 0.003)
        self.declare_parameter("pinch_rescale_k", 0.6)
        self.declare_parameter("solver", "projgd")   # projgd | slsqp
        self.declare_parameter("max_iter", 4)
        self.declare_parameter("ftol", 1e-4)
        self.declare_parameter("gtol", 1e-2)
        self.declare_parameter("projgd_lr0", 0.003)

        # Rate
        self.declare_parameter("publish_rate_hz", 60.0)
        # Hard per-tick joint step cap after IK (rad). Prevents commanded
        # reference from jumping more than this many radians per publish tick
        # regardless of the optimizer's choice. Set <= 0 to disable.
        self.declare_parameter("max_step_per_tick_rad", 0.12)

        # Contact-aware (b) layer
        self.declare_parameter("contact_aware", False)
        self.declare_parameter("contact_c_lo", 0.30)
        self.declare_parameter("contact_c_hi", 0.80)
        self.declare_parameter("contact_max_blend", 0.85)
        self.declare_parameter("contact_velocity_clamp_rad_s", 1.5)

        self._side = str(self.get_parameter("hand_side").value).lower()
        if self._side not in ("right", "left"):
            raise ValueError("hand_side must be 'right' or 'left'")

        self._joint_names = RIGHT_JOINT_NAMES if self._side == "right" else LEFT_JOINT_NAMES
        self._postproc = POSTPROC_RIGHT if self._side == "right" else POSTPROC_LEFT

        self._in_topic = self.get_parameter("input_topic").value
        self._out_topic = self.get_parameter("output_topic").value
        self._expected_side = str(self.get_parameter("expected_side").value).lower()

        self._cmc_mode = str(self.get_parameter("thumb_cmc_mode").value).lower()
        self._cmc_fixed = float(self.get_parameter("thumb_cmc_fixed_value_rad").value)
        self._cmc_offset = float(self.get_parameter("thumb_cmc_offset_deg").value)
        self._cmc_gain_stretch = float(self.get_parameter("thumb_cmc_gain_stretch").value)
        self._cmc_gain_spread = float(self.get_parameter("thumb_cmc_gain_spread").value)

        self._calib = np.array([float(x) for x in self.get_parameter("calib").value], dtype=float)
        self._dir = np.array([float(x) for x in self.get_parameter("dir_sign").value], dtype=float)
        if self._calib.size != 20 or self._dir.size != 20:
            raise ValueError("calib / dir_sign must have exactly 20 entries")

        urdf_path = str(self.get_parameter("urdf_path").value)
        if not urdf_path:
            urdf_path = _default_urdf_path(self._side)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        weights = AObjectiveWeights(
            prior=float(self.get_parameter("w_prior").value),
            velocity=float(self.get_parameter("w_velocity").value),
            orient=float(self.get_parameter("w_orient").value),
            pinch=float(self.get_parameter("w_pinch").value),
            pinch_close_ref_m=float(self.get_parameter("pinch_close_ref_m").value),
            pinch_far_ref_m=float(self.get_parameter("pinch_far_ref_m").value),
            pinch_target_min_m=float(self.get_parameter("pinch_target_min_m").value),
            pinch_rescale_k=float(self.get_parameter("pinch_rescale_k").value),
        )
        self._cfg = _build_cfg(self._side, urdf_path, weights)
        self._cfg.solver = str(self.get_parameter("solver").value).lower()
        self._cfg.max_iter = int(self.get_parameter("max_iter").value)
        self._cfg.ftol = float(self.get_parameter("ftol").value)
        self._cfg.gtol = float(self.get_parameter("gtol").value)
        self._cfg.projgd_lr0 = float(self.get_parameter("projgd_lr0").value)

        self._max_step = float(self.get_parameter("max_step_per_tick_rad").value)

        self._contact_cfg = ContactConfig(
            enabled=bool(self.get_parameter("contact_aware").value),
            c_lo=float(self.get_parameter("contact_c_lo").value),
            c_hi=float(self.get_parameter("contact_c_hi").value),
            max_blend=float(self.get_parameter("contact_max_blend").value),
            velocity_clamp_rad_s=float(self.get_parameter("contact_velocity_clamp_rad_s").value),
        )

        self._last_glove: Optional[ManusGlove] = None
        self._q_prev: Optional[np.ndarray] = None
        self._q_meas: Optional[np.ndarray] = None
        self._contact: Optional[np.ndarray] = None
        self._last_publish_stamp: Optional[float] = None

        self._sub_glove = self.create_subscription(
            ManusGlove, self._in_topic, self._on_glove, 10)
        self._pub_cmd = self.create_publisher(MultiDOFCommand, self._out_topic, 10)

        if self._contact_cfg.enabled:
            js_topic = str(self.get_parameter("joint_states_topic").value)
            cl_topic = str(self.get_parameter("contact_level_topic").value)
            self.create_subscription(JointState, js_topic, self._on_js, 10)
            self.create_subscription(Float32MultiArray, cl_topic, self._on_contact, 10)
            self.get_logger().info(
                f"contact_aware ON: joint_states={js_topic} contact={cl_topic}")

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"manus_dg5f_sota_retargeting_a_good[{self._side}]: {self._in_topic} -> {self._out_topic} "
            f"| urdf={os.path.basename(urdf_path)} "
            f"| solver={self._cfg.solver} max_iter={self._cfg.max_iter} "
            f"| weights prior={weights.prior} vel={weights.velocity} "
            f"orient={weights.orient} pinch={weights.pinch} "
            f"| rate={rate:.0f} Hz"
        )

    def _on_glove(self, msg: ManusGlove):
        side = (msg.side or "").lower()
        if self._expected_side != "any" and side != self._expected_side:
            return
        self._last_glove = msg

    def _on_js(self, msg: JointState):
        # Map joint_states to our 20-slot order.
        q = np.zeros(20)
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(self._joint_names):
            idx = name_to_idx.get(name)
            if idx is not None and idx < len(msg.position):
                q[i] = float(msg.position[idx])
        self._q_meas = q

    def _on_contact(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) != 20:
            return
        self._contact = np.array(data, dtype=float)

    def _ergo_dict(self, msg: ManusGlove) -> Dict[str, float]:
        ergo = {e.type: float(e.value) for e in msg.ergonomics}
        for prefix in ("Right", "Left"):
            if any(k.startswith(prefix) for k in ergo):
                ergo = {(k[len(prefix):] if k.startswith(prefix) else k): v
                        for k, v in ergo.items()}
                break
        return ergo

    def _tick(self):
        msg = self._last_glove
        if msg is None:
            return
        ergo = self._ergo_dict(msg)


        cmc_rad = compute_thumb_cmc(
            mode=self._cmc_mode,
            spread_deg=float(ergo.get("ThumbMCPSpread", 0.0)),
            stretch_deg=float(ergo.get("ThumbMCPStretch", 0.0)),
            fixed_value_rad=self._cmc_fixed,
            offset_deg=self._cmc_offset,
            gain_stretch=self._cmc_gain_stretch,
            gain_spread=self._cmc_gain_spread,
        )
        q0 = ergo_to_q0_rad(ergo, self._calib, self._dir, self._postproc, cmc_rad)

        q_opt, _info = solve_ik(self._cfg, q0, self._q_prev)


        # Per-tick step cap (smooth out SLSQP jumps when ergo changes fast).
        if self._q_prev is not None and self._max_step > 0.0:
            delta = np.clip(q_opt - self._q_prev, -self._max_step, self._max_step)
            q_opt = self._q_prev + delta

        # (b)-layer
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._last_publish_stamp is None else max(0.0, now - self._last_publish_stamp)
        q_cmd = modulate(q_opt, self._q_prev if self._q_prev is not None else q_opt,
                         self._q_meas, self._contact, self._contact_cfg, dt)

        out = MultiDOFCommand()
        out.dof_names = list(self._joint_names)
        out.values = q_cmd.tolist()
        out.values_dot = [0.0] * 20
        self._pub_cmd.publish(out)

        self._q_prev = q_cmd
        self._last_publish_stamp = now


def main(args=None):
    rclpy.init(args=args)
    node = ManusDg5fSotaRetargetA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
