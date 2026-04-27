"""Launch only the SOTA-A retarget node with its yaml auto-loaded.

Use when Manus driver + DG side already run elsewhere. Typical use:
  ros2 launch manus_dg5f_sota_retarget_a retarget_only.launch.py hand_side:=right
  ros2 launch manus_dg5f_sota_retarget_a retarget_only.launch.py hand_side:=left

Pass `params_file:=/abs/path/to/your.yaml` to override the in-package
yaml — e.g. when iterating with dg5f_calib_wizard's output. The path
is loaded BEFORE the in-package yaml so values in `params_file` win.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument("hand_side", default_value="right",
                              description="right | left"),
        DeclareLaunchArgument("grip_mode", default_value="tiptotip",
                              description="tiptotip | pad"),
        DeclareLaunchArgument("contact_aware", default_value="false",
                              description="true | false — enable (b)-layer"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="absolute path to override yaml "
                                          "(empty => use in-package yaml)"),
    ]
    hand_side = LaunchConfiguration("hand_side")
    grip_mode = LaunchConfiguration("grip_mode")
    contact_aware = LaunchConfiguration("contact_aware")
    params_file = LaunchConfiguration("params_file")

    # Pick {right,left}_hand[_pad].yaml based on hand_side + grip_mode.
    yaml_name = PythonExpression([
        '"', hand_side, '_hand" + ("_pad" if "', grip_mode, '" == "pad" else "") + ".yaml"'
    ])
    cfg_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_sota_retarget_a"), "config", yaml_name
    ])

    node_name = PythonExpression([
        '"manus_dg5f_sota_retarget_a_" + "', hand_side, '"'
    ])

    has_override = IfCondition(PythonExpression(['"', params_file, '" != ""']))
    no_override = UnlessCondition(PythonExpression(['"', params_file, '" != ""']))

    # Two Node entries; one fires depending on whether params_file is set.
    retarget_default = Node(
        package="manus_dg5f_sota_retarget_a",
        executable="retarget_node",
        name=node_name,
        output="screen",
        parameters=[cfg_yaml, {"contact_aware": contact_aware}],
        condition=no_override,
    )
    retarget_override = Node(
        package="manus_dg5f_sota_retarget_a",
        executable="retarget_node",
        name=node_name,
        output="screen",
        parameters=[cfg_yaml, params_file, {"contact_aware": contact_aware}],
        condition=has_override,
    )

    return LaunchDescription(declared + [retarget_default, retarget_override])
