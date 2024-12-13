from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    start_sequence_node = Node(
        package="pulse_sequence_controller",
        executable="start_pulse_sequence"
    )

    ld.add_action(start_sequence_node)
    return ld
