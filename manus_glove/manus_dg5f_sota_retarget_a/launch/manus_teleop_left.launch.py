"""End-to-end Manus -> DG5F left-hand SOTA-A teleop launch.

Mirror of manus_teleop_right.launch.py for the left hand (glove id 0).
If both hands use manus_source:=real they will fight over the Manus USB
device — set one to manus_source:=external and run the driver separately.
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
        DeclareLaunchArgument("manus_source", default_value="sim",
                              description="sim | real | external"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
                              description="true = DG mock, false = real PID driver"),
        DeclareLaunchArgument("contact_aware", default_value="false",
                              description="enable (b)-layer"),
    ]

    manus_source = LaunchConfiguration("manus_source")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    contact_aware = LaunchConfiguration("contact_aware")

    is_sim = IfCondition(PythonExpression(['"', manus_source, '" == "sim"']))
    is_real = IfCondition(PythonExpression(['"', manus_source, '" == "real"']))

    cfg_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_sota_retarget_a"), "config", "left_hand.yaml"
    ])

    sim_glove = Node(
        package="manus_glove_sim",
        executable="sim_glove",
        name="manus_sim_slider_left",
        output="screen",
        parameters=[{"topic": "/manus_glove_0", "side": "Left", "glove_id": 0}],
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
        package="manus_dg5f_sota_retarget_a",
        executable="retarget_node",
        name="manus_dg5f_sota_retarget_a_left",
        output="screen",
        parameters=[cfg_yaml, {"contact_aware": contact_aware}],
    )

    dg_mock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("dg5f_driver"), "launch",
                "dg5f_left_mock.launch.py"
            ])
        ]),
        condition=IfCondition(use_mock_hardware),
    )

    dg_pid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("dg5f_driver"), "launch",
                "dg5f_left_pid_all_controller.launch.py"
            ])
        ]),
        condition=UnlessCondition(use_mock_hardware),
    )

    return LaunchDescription(
        declared + [sim_glove, real_manus, retarget, dg_mock, dg_pid]
    )
