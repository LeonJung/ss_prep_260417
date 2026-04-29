"""End-to-end Manus → DG5F (left) teleop with grasp_mode mux node.

Same shape as manus_dg5f_retarget/launch/manus_teleop_left.launch.py
but inserts manus_dg5f_grasp_mode between the retarget node and the
DG driver:

  glove   -> retarget -> .../reference_free
                                          \\-> grasp_mode_node -> .../reference -> DG
  /grasp_mode (std_msgs/String) ----------------/

Switch modes at runtime:
  ros2 topic pub --once /grasp_mode std_msgs/String "data: 'free'"
  ros2 topic pub --once /grasp_mode std_msgs/String "data: 'key_grip'"

Args (mirrors manus_dg5f_retarget/launch/manus_teleop_left.launch.py):
  manus_source       sim (default) | real | external
  use_mock_hardware  true (default) | false
  thumb_cmc_mode     fixed | coupled | raw_nodes_ik
  default_mode       free (default) — mode applied at startup
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


SIDE = "left"
GLOVE_TOPIC = "/manus_glove_0"
REF_FREE_TOPIC = f"/dg5f_{SIDE}/lj_dg_pospid/reference_free"
REF_OUT_TOPIC = f"/dg5f_{SIDE}/lj_dg_pospid/reference"


def generate_launch_description():
    declared = [
        DeclareLaunchArgument("manus_source", default_value="sim",
                              description="sim | real | external"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("thumb_cmc_mode", default_value="fixed"),
        DeclareLaunchArgument("default_mode", default_value="free"),
    ]
    manus_source = LaunchConfiguration("manus_source")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    thumb_cmc_mode = LaunchConfiguration("thumb_cmc_mode")
    default_mode = LaunchConfiguration("default_mode")

    is_sim = IfCondition(PythonExpression(['"', manus_source, '" == "sim"']))
    is_real = IfCondition(PythonExpression(['"', manus_source, '" == "real"']))

    calib_yaml = PathJoinSubstitution([
        FindPackageShare("manus_dg5f_retarget"),
        "config", "left_hand_calib.yaml"
    ])

    sim_glove = Node(
        package="manus_glove_sim",
        executable="sim_glove",
        name="manus_sim_slider_left",
        output="screen",
        parameters=[{"topic": GLOVE_TOPIC, "side": "Left", "glove_id": 0}],
        condition=is_sim,
    )

    real_manus = Node(
        package="manus_ros2",
        executable="manus_data_publisher",
        name="manus_data_publisher",
        output="screen",
        condition=is_real,
    )

    # Base retarget node — its output is remapped to the *_free topic
    # so the grasp_mode_node can re-publish the final reference.
    retarget = Node(
        package="manus_dg5f_retarget",
        executable="retarget_node",
        name="manus_dg5f_retarget_left",
        output="screen",
        parameters=[
            calib_yaml,
            {"thumb_cmc_mode": thumb_cmc_mode,
             "output_topic": REF_FREE_TOPIC},
        ],
    )

    grasp_mode = Node(
        package="manus_dg5f_grasp_mode",
        executable="grasp_mode_node",
        name="manus_dg5f_grasp_mode_left",
        output="screen",
        parameters=[{
            "hand_side": SIDE,
            "glove_topic": GLOVE_TOPIC,
            "reference_in_topic": REF_FREE_TOPIC,
            "reference_out_topic": REF_OUT_TOPIC,
            "default_mode": default_mode,
            "expected_side": "left",
        }],
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
        declared + [sim_glove, real_manus, retarget, grasp_mode,
                    dg_mock, dg_pid]
    )
