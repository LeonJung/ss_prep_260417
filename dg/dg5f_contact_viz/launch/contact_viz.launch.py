# Launch contact_monitor (JointState -> contact_level) and the Qt viz widget.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    thresholds = PathJoinSubstitution(
        [FindPackageShare("dg5f_contact_viz"), "config", "contact_thresholds.yaml"]
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

    return LaunchDescription([monitor, viz])
