"""End-to-end Manus -> DG5F right-hand teleop launch.

Brings up:
  * manus_glove_sim (Qt slider) — optional, on by default for pre-hardware tests
  * manus_dg5f_retarget_node
  * dg5f_driver mock or real pid_all controller (for DG side)

Args:
  use_sim_glove       : true (default) -> run Qt slider UI publishing /manus_glove_1
                        false -> expect a real Manus driver to publish /manus_glove_1
  use_mock_hardware   : true (default) -> dg5f_right_mock.launch.py
                        false -> dg5f_right_pid_all_controller.launch.py (real hw)
  thumb_cmc_mode      : fixed | coupled | raw_nodes_ik
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument("use_sim_glove", default_value="true"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("thumb_cmc_mode", default_value="fixed"),
    ]

    use_sim_glove = LaunchConfiguration("use_sim_glove")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    thumb_cmc_mode = LaunchConfiguration("thumb_cmc_mode")

    calib_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_retarget"),
        "config", "right_hand_calib.yaml"
    ])

    sim_glove = Node(
        package="manus_glove_sim",
        executable="sim_glove",
        name="manus_sim_slider",
        output="screen",
        condition=IfCondition(use_sim_glove),
    )

    retarget = Node(
        package="manus_dg5f_retarget",
        executable="retarget_node",
        name="manus_dg5f_retarget",
        output="screen",
        parameters=[calib_yaml, {"thumb_cmc_mode": thumb_cmc_mode}],
    )

    dg_mock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("dg5f_driver"), "launch",
                "dg5f_right_mock.launch.py"
            ])
        ]),
        condition=IfCondition(use_mock_hardware),
    )

    dg_pid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("dg5f_driver"), "launch",
                "dg5f_right_pid_all_controller.launch.py"
            ])
        ]),
        condition=UnlessCondition(use_mock_hardware),
    )

    return LaunchDescription(declared + [sim_glove, retarget, dg_mock, dg_pid])
