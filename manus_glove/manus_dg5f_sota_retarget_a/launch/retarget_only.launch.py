"""Launch only the SOTA-A retarget node with its yaml auto-loaded.

Use when Manus driver + DG side already run elsewhere. Typical use:
  ros2 launch manus_dg5f_sota_retarget_a retarget_only.launch.py hand_side:=right
  ros2 launch manus_dg5f_sota_retarget_a retarget_only.launch.py hand_side:=left
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
        DeclareLaunchArgument("hand_side", default_value="right",
                              description="right | left"),
        DeclareLaunchArgument("grip_mode", default_value="tiptotip",
                              description="tiptotip | pad"),
        DeclareLaunchArgument("contact_aware", default_value="false",
                              description="true | false — enable (b)-layer"),
    ]
    hand_side = LaunchConfiguration("hand_side")
    grip_mode = LaunchConfiguration("grip_mode")
    contact_aware = LaunchConfiguration("contact_aware")

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

    retarget = Node(
        package="manus_dg5f_sota_retarget_a",
        executable="retarget_node",
        name=node_name,
        output="screen",
        parameters=[cfg_yaml, {"contact_aware": contact_aware}],
    )

    return LaunchDescription(declared + [retarget])
