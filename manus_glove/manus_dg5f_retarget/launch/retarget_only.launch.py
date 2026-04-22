"""Launch only the retarget node with its yaml auto-loaded.

Use this when the Manus driver and the DG side are already running in
other terminals (e.g. `manus_ros2 manus_data_publisher` +
`dg5f_driver dg5f_{right,left}_pid_all_controller.launch.py`) and you
just want the bridge on top.

Args:
  hand_side      : right (default) | left
                   Picks right_hand_calib.yaml or left_hand_calib.yaml.
  thumb_cmc_mode : fixed | coupled | raw_nodes_ik (overrides yaml)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "hand_side", default_value="right",
            description="right | left"
        ),
        DeclareLaunchArgument(
            "thumb_cmc_mode", default_value="fixed",
            description="fixed | coupled | raw_nodes_ik"
        ),
    ]
    hand_side = LaunchConfiguration("hand_side")
    thumb_cmc_mode = LaunchConfiguration("thumb_cmc_mode")

    yaml_name = PythonExpression([
        '"right_hand_calib.yaml" if "', hand_side, '" == "right" else "left_hand_calib.yaml"'
    ])
    calib_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_retarget"), "config", yaml_name
    ])

    node_name = PythonExpression([
        '"manus_dg5f_retarget_" + "', hand_side, '"'
    ])

    retarget = Node(
        package="manus_dg5f_retarget",
        executable="retarget_node",
        name=node_name,
        output="screen",
        parameters=[calib_yaml, {"thumb_cmc_mode": thumb_cmc_mode}],
    )

    return LaunchDescription(declared + [retarget])
