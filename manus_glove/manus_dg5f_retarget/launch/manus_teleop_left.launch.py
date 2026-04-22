"""End-to-end Manus -> DG5F left-hand teleop launch.

Mirror of manus_teleop_right.launch.py for the left hand. Consumes
glove id 0 (/manus_glove_0) and publishes to /dg5f_left/lj_dg_pospid/
reference, using left_hand_calib.yaml.

Args:
  manus_source      : sim (default) | real | external
  use_mock_hardware : true (default) | false
  thumb_cmc_mode    : fixed | coupled | raw_nodes_ik

If you run both the left and right teleop launches at the same time and
both have manus_source:=real, two copies of the Manus driver will fight
for the same USB device. Use manus_source:=external on one of the two
(and run `ros2 run manus_ros2 manus_data_publisher` in its own terminal).
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
        "config", "left_hand_calib.yaml"
    ])

    # Qt slider sim — publishes /manus_glove_0 for the left hand.
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
        package="manus_dg5f_retarget",
        executable="retarget_node",
        name="manus_dg5f_retarget_left",
        output="screen",
        parameters=[calib_yaml, {"thumb_cmc_mode": thumb_cmc_mode}],
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
