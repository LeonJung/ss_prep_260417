# DG5F Right Hand bringup
# Effort (force-feedback) control + optional mock hardware + optional fingertip F/T broadcasters.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


NAMESPACE = "dg5f_right"
JOINT_CONTROLLER = "effort_controller"
FINGERTIP_BROADCASTERS = [
    "fingertip_1_broadcaster",
    "fingertip_2_broadcaster",
    "fingertip_3_broadcaster",
    "fingertip_4_broadcaster",
    "fingertip_5_broadcaster",
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Run against mock_components::GenericSystem without real DG5F."
        ),
        DeclareLaunchArgument(
            "delto_ip", default_value="169.254.186.72",
            description="DG5F TCP IP address (ignored when use_mock_hardware:=true)."
        ),
        DeclareLaunchArgument(
            "delto_port", default_value="502",
            description="DG5F TCP port (ignored when use_mock_hardware:=true)."
        ),
        DeclareLaunchArgument(
            "fingertip_sensor", default_value="false",
            description="Enable fingertip F/T sensor in hardware interface."
        ),
        DeclareLaunchArgument(
            "ft_broadcaster", default_value="false",
            description="Spawn 5 fingertip F/T broadcasters."
        ),
        DeclareLaunchArgument(
            "io", default_value="false",
            description="Enable GPIO interfaces."
        ),
    ]

    use_mock = LaunchConfiguration("use_mock_hardware")
    ip = LaunchConfiguration("delto_ip")
    port = LaunchConfiguration("delto_port")
    fingertip_sensor = LaunchConfiguration("fingertip_sensor")
    ft_broadcaster = LaunchConfiguration("ft_broadcaster")
    io = LaunchConfiguration("io")

    xacro_real = PathJoinSubstitution(
        [FindPackageShare("dg5f_driver"), "urdf", "dg5f_right_ros2_control.xacro"]
    )
    xacro_mock = PathJoinSubstitution(
        [FindPackageShare("dg5f_driver"), "urdf", "dg5f_right_mock.xacro"]
    )

    robot_description_real = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_real,
            " ", "delto_ip:=", ip,
            " ", "delto_port:=", port,
            " ", "fingertip_sensor:=", fingertip_sensor,
            " ", "io:=", io,
        ])
    }
    robot_description_mock = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_mock,
        ])
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("dg5f_hand_bringup"), "config", "dg5f_right_bilateral.yaml"]
    )
    ft_broadcaster_yaml = PathJoinSubstitution(
        [FindPackageShare("dg5f_driver"), "config", "dg5f_right_ft_broadcaster.yaml"]
    )

    def control_node(params, when):
        return Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=NAMESPACE,
            parameters=params,
            remappings=[("~/robot_description", f"/{NAMESPACE}/robot_description")],
            output="screen",
            condition=when,
        )

    real_no_ft = IfCondition(PythonExpression(
        ["not ", use_mock, " and not ", ft_broadcaster]
    ))
    real_with_ft = IfCondition(PythonExpression(
        ["not ", use_mock, " and ", ft_broadcaster]
    ))

    control_real = control_node([controllers_yaml], real_no_ft)
    control_real_ft = control_node([controllers_yaml, ft_broadcaster_yaml], real_with_ft)
    control_mock = control_node([controllers_yaml], IfCondition(use_mock))

    rsp_real = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        output="screen",
        parameters=[robot_description_real],
        condition=UnlessCondition(use_mock),
    )
    rsp_mock = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        output="screen",
        parameters=[robot_description_mock],
        condition=IfCondition(use_mock),
    )

    def spawner(name, when=None):
        kwargs = dict(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", f"/{NAMESPACE}/controller_manager"],
            output="screen",
        )
        if when is not None:
            kwargs["condition"] = when
        return Node(**kwargs)

    jsb_spawner = spawner("joint_state_broadcaster")
    effort_spawner = spawner(JOINT_CONTROLLER)
    ft_spawners = [spawner(n, IfCondition(ft_broadcaster)) for n in FINGERTIP_BROADCASTERS]

    return LaunchDescription(
        declared + [
            control_real, control_real_ft, control_mock,
            rsp_real, rsp_mock,
            jsb_spawner, effort_spawner,
            *ft_spawners,
        ]
    )
