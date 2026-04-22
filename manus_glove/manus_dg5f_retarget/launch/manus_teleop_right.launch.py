"""End-to-end Manus -> DG5F right-hand teleop launch.

Brings up:
  * Manus source: Qt slider sim, real Manus driver, or nothing (external)
  * manus_dg5f_retarget (yaml auto-loaded from this package's share dir)
  * DG5F side: mock ros2_control or real pid_all_controller

Args:
  manus_source      : sim (default) | real | external
                      sim      -> run manus_glove_sim Qt slider
                      real     -> run manus_ros2 manus_data_publisher
                                  (commercial Manus SDK must be installed)
                      external -> launch nothing on the Manus side
                                  (user is running a driver elsewhere)
  use_mock_hardware : true (default) | false
                      true     -> dg5f_right_mock.launch.py
                      false    -> dg5f_right_pid_all_controller.launch.py
  thumb_cmc_mode    : fixed | coupled | raw_nodes_ik
                      overrides the value in right_hand_calib.yaml

Typical real-hardware usage:
  ros2 launch manus_dg5f_retarget manus_teleop_right.launch.py \\
       manus_source:=real use_mock_hardware:=false

Notes:
  * The retargeter's calib yaml is loaded automatically from this
    package's installed share/manus_dg5f_retarget/config/right_hand_calib.yaml
  * To edit the yaml, open
    ~/hand_ws/src/manus_glove/manus_dg5f_retarget/config/right_hand_calib.yaml
    (colcon build's --symlink-install makes the installed copy a symlink,
    so edits apply immediately on relaunch)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "manus_source", default_value="sim",
            description="sim | real | external"
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="true = DG mock, false = real DG via pid_all_controller"
        ),
        DeclareLaunchArgument(
            "thumb_cmc_mode", default_value="fixed",
            description="fixed | coupled | raw_nodes_ik"
        ),
    ]

    manus_source = LaunchConfiguration("manus_source")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    thumb_cmc_mode = LaunchConfiguration("thumb_cmc_mode")

    is_sim = IfCondition(PythonExpression(['"', manus_source, '" == "sim"']))
    is_real = IfCondition(PythonExpression(['"', manus_source, '" == "real"']))

    calib_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_retarget"),
        "config", "right_hand_calib.yaml"
    ])

    sim_glove = Node(
        package="manus_glove_sim",
        executable="sim_glove",
        name="manus_sim_slider",
        output="screen",
        condition=is_sim,
    )

    real_manus = Node(
        package="manus_ros2",
        executable="manus_data_publisher",
        name="manus_data_publisher",
        output="screen",
        condition=is_real,
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

    return LaunchDescription(
        declared + [sim_glove, real_manus, retarget, dg_mock, dg_pid]
    )
