# Pre-hardware simulation stack: fake driver + contact monitor + Qt viz.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "scenario", default_value="grip_cycle",
            description="idle | sweep | grip_cycle"),
        DeclareLaunchArgument(
            "rate_hz", default_value="50.0"),
        DeclareLaunchArgument(
            "with_viz", default_value="true",
            description="Launch the Qt contact viz alongside the monitor."),
    ]

    scenario = LaunchConfiguration("scenario")
    rate_hz = LaunchConfiguration("rate_hz")

    thresholds = PathJoinSubstitution(
        [FindPackageShare("dg5f_contact_viz"), "config", "contact_thresholds.yaml"]
    )

    sim = Node(
        package="dg5f_hand_bringup",
        executable="dg5f_sim_driver.py",
        name="dg5f_sim_driver",
        output="screen",
        parameters=[{
            "topic": "/dg5f_right/joint_states",
            "scenario": scenario,
            "rate_hz": rate_hz,
        }],
    )

    monitor = Node(
        package="dg5f_contact_viz",
        executable="contact_monitor",
        name="contact_monitor",
        output="screen",
        parameters=[thresholds],
    )

    viz = Node(
        package="dg5f_contact_viz",
        executable="contact_viz",
        name="contact_viz",
        output="screen",
    )

    return LaunchDescription(declared + [sim, monitor, viz])
