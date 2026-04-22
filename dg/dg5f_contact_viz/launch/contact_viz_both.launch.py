"""Bimanual contact: both monitors + a single widget in 'both' display mode.

Renders the left hand on the left half of the window (mirrored) and the
right hand on the right half. Requires both DG5F drivers to be publishing
their joint_states.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    right_yaml = PathJoinSubstitution([
        FindPackageShare("dg5f_contact_viz"),
        "config", "contact_thresholds.yaml"
    ])
    left_yaml = PathJoinSubstitution([
        FindPackageShare("dg5f_contact_viz"),
        "config", "contact_thresholds_left.yaml"
    ])

    right_monitor = Node(
        package="dg5f_contact_viz",
        executable="contact_monitor",
        name="contact_monitor_right",
        output="screen",
        parameters=[right_yaml],
    )
    left_monitor = Node(
        package="dg5f_contact_viz",
        executable="contact_monitor",
        name="contact_monitor_left",
        output="screen",
        parameters=[left_yaml],
    )

    viz = Node(
        package="dg5f_contact_viz",
        executable="contact_viz",
        name="contact_viz",
        output="screen",
        parameters=[{"display_mode": "both"}],
    )

    return LaunchDescription([right_monitor, left_monitor, viz])
