"""Mux Manus → DG5F retargeter through preset grasp modes.

Topology (handled by the launch file):

  /manus_glove_<id> -> [manus_dg5f_retarget] -> reference_free
                       [grasp_mode_node]    -> reference  -> DG5F driver
  /grasp_mode (std_msgs/String) ----------------^

Modes are defined in dg5f_mujoco_sim/config/grasp_modes_<side>.yaml.
Format (one entry per mode):

  modes:
    key_grip:
      q_target:        [20 floats]    # fully closed config
      variable_axes:                  # optional
        - {slot: 2, from: 0.0, drive: thumb_curl}
        - ...
      description, tip_pose, timestamp: ignored at runtime

When the active mode is `free`, the node forwards the base
retargeter output verbatim (so Manus → DG behaves exactly like it
does without this node). When the active mode matches an entry in
the yaml, the node:
  - starts from `q_target`
  - for each variable axis, replaces q[slot] with
    `from + drive(glove) * (q_target[slot] - from)`
  - publishes the result.

Switch modes by publishing on /grasp_mode (std_msgs/String):
  ros2 topic pub --once /grasp_mode std_msgs/String "data: 'key_grip'"
  ros2 topic pub --once /grasp_mode std_msgs/String "data: 'free'"
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from rclpy.node import Node

from control_msgs.msg import MultiDOFCommand
from manus_ros2_msgs.msg import ManusGlove
from std_msgs.msg import String

from .drive_signals import compute_drives


def _default_yaml(side: str) -> str:
    """Locate config/grasp_modes_<side>.yaml in dg5f_mujoco_sim share."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("dg5f_mujoco_sim"))
                   / "config" / f"grasp_modes_{side}.yaml")
    except Exception:
        return ""


class GraspModeNode(Node):
    def __init__(self):
        super().__init__("manus_dg5f_grasp_mode")

        self.declare_parameter("hand_side", "left")
        self.declare_parameter("glove_topic", "/manus_glove_0")
        self.declare_parameter(
            "reference_in_topic",
            "/dg5f_left/lj_dg_pospid/reference_free")
        self.declare_parameter(
            "reference_out_topic",
            "/dg5f_left/lj_dg_pospid/reference")
        self.declare_parameter("grasp_mode_topic", "/grasp_mode")
        self.declare_parameter("grasp_modes_yaml", "")
        self.declare_parameter("default_mode", "free")
        self.declare_parameter("expected_side", "any")  # right | left | any

        self._side = str(self.get_parameter("hand_side").value).lower()
        if self._side not in ("left", "right"):
            raise ValueError(f"hand_side must be left|right, got {self._side!r}")

        glove_topic = str(self.get_parameter("glove_topic").value)
        ref_in_topic = str(self.get_parameter("reference_in_topic").value)
        ref_out_topic = str(self.get_parameter("reference_out_topic").value)
        mode_topic = str(self.get_parameter("grasp_mode_topic").value)
        yaml_path = str(self.get_parameter("grasp_modes_yaml").value
                        or _default_yaml(self._side))
        self._expected_side = str(
            self.get_parameter("expected_side").value).lower()

        self._modes: Dict[str, Any] = self._load_modes(yaml_path)
        self.get_logger().info(
            f"loaded {len(self._modes)} mode(s) from {yaml_path}: "
            f"{sorted(self._modes)}")

        self._mode_name: str = str(
            self.get_parameter("default_mode").value)
        if self._mode_name not in (("free",) + tuple(self._modes)):
            self.get_logger().warn(
                f"default_mode {self._mode_name!r} unknown; "
                f"falling back to 'free'")
            self._mode_name = "free"

        self._latest_base_values: Optional[List[float]] = None
        self._latest_dof_names: Optional[List[str]] = None
        self._latest_glove: Optional[ManusGlove] = None

        self._sub_ref = self.create_subscription(
            MultiDOFCommand, ref_in_topic, self._on_ref_in, 10)
        self._sub_glove = self.create_subscription(
            ManusGlove, glove_topic, self._on_glove, 10)
        self._sub_mode = self.create_subscription(
            String, mode_topic, self._on_mode, 10)
        self._pub = self.create_publisher(MultiDOFCommand, ref_out_topic, 10)

        self.get_logger().info(
            f"side={self._side}  mode={self._mode_name}  "
            f"in={ref_in_topic}  glove={glove_topic}  out={ref_out_topic}")

    @staticmethod
    def _load_modes(yaml_path: str) -> Dict[str, Any]:
        if not yaml_path:
            return {}
        p = Path(yaml_path)
        if not p.exists():
            return {}
        with open(p) as f:
            doc = yaml.safe_load(f) or {}
        modes = doc.get("modes") or {}
        out: Dict[str, Any] = {}
        for name, m in modes.items():
            q = m.get("q_target") or m.get("q")  # back-compat
            if not isinstance(q, list) or len(q) != 20:
                continue
            out[name] = dict(
                q_target=[float(v) for v in q],
                variable_axes=list(m.get("variable_axes") or []),
            )
        return out

    def _on_mode(self, msg: String):
        new_mode = (msg.data or "").strip()
        if new_mode == self._mode_name:
            return
        if new_mode == "free" or new_mode in self._modes:
            self._mode_name = new_mode
            self.get_logger().info(f"mode -> {new_mode}")
        else:
            self.get_logger().warn(
                f"unknown mode {new_mode!r}; staying in {self._mode_name}; "
                f"available: free, {', '.join(sorted(self._modes))}")

    def _on_ref_in(self, msg: MultiDOFCommand):
        # Cache base retargeter output. Only published when mode==free.
        if self._latest_dof_names is None:
            self._latest_dof_names = list(msg.dof_names)
        self._latest_base_values = list(msg.values)
        if self._mode_name == "free":
            self._publish(list(msg.values), list(msg.dof_names))

    def _on_glove(self, msg: ManusGlove):
        if (self._expected_side != "any"
                and (msg.side or "").lower() != self._expected_side):
            return
        self._latest_glove = msg
        if self._mode_name != "free" and self._mode_name in self._modes:
            self._publish_mode_pose()

    def _publish_mode_pose(self):
        if self._latest_dof_names is None:
            # We need the dof_names from the base retargeter to publish a
            # well-formed MultiDOFCommand. Wait until the first message.
            return
        defn = self._modes[self._mode_name]
        q = list(defn["q_target"])
        if defn["variable_axes"] and self._latest_glove is not None:
            ergo = {e.type: float(e.value) for e in self._latest_glove.ergonomics}
            drives = compute_drives(ergo)
            for ax in defn["variable_axes"]:
                slot = int(ax["slot"])
                from_v = float(ax["from"])
                target_v = q[slot]
                drive = str(ax["drive"])
                f = float(drives.get(drive, 0.0))
                q[slot] = from_v + f * (target_v - from_v)
        self._publish(q, self._latest_dof_names)

    def _publish(self, values: List[float], names: List[str]):
        out = MultiDOFCommand()
        out.dof_names = list(names)
        out.values = list(values)
        out.values_dot = [0.0] * len(values)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GraspModeNode()
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
