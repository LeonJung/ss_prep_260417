"""Left-hand contact monitor + single-hand viz widget (mirrored render)."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    thresholds = PathJoinSubstitution([
        FindPackageShare("dg5f_contact_viz"),
        "config", "contact_thresholds_left.yaml"
    ])

    monitor = Node(
        package="dg5f_contact_viz",
        executable="contact_monitor",
        name="contact_monitor_left",
        output="screen",
        parameters=[thresholds],
    )

    viz = Node(
        package="dg5f_contact_viz",
        executable="contact_viz",
        name="contact_viz",
        output="screen",
        parameters=[{"display_mode": "left"}],
    )

    return LaunchDescription([monitor, viz])
