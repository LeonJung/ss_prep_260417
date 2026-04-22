"""Launch only the retarget node with its yaml auto-loaded.

Use this when the Manus driver and the DG side are already running in
other terminals (e.g. you have `manus_ros2 manus_data_publisher` and
`dg5f_driver dg5f_right_pid_all_controller.launch.py` up separately).

Args:
  thumb_cmc_mode : fixed | coupled | raw_nodes_ik (overrides yaml)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "thumb_cmc_mode", default_value="fixed",
            description="fixed | coupled | raw_nodes_ik"
        ),
    ]
    thumb_cmc_mode = LaunchConfiguration("thumb_cmc_mode")

    calib_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_retarget"),
        "config", "right_hand_calib.yaml"
    ])

    retarget = Node(
        package="manus_dg5f_retarget",
        executable="retarget_node",
        name="manus_dg5f_retarget",
        output="screen",
        parameters=[calib_yaml, {"thumb_cmc_mode": thumb_cmc_mode}],
    )

    return LaunchDescription(declared + [retarget])
