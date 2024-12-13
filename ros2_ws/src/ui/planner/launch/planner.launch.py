from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    nodes = [
        Node(
            package="planner",
            executable="add_target"
        ),
        Node(
            package="planner",
            executable="remove_target"
        ),
        Node(
            package="planner",
            executable="toggle_select_target"
        ),
        Node(
            package="planner",
            executable="set_target"
        ),
        Node(
            package="planner",
            executable="rename_target"
        ),
        Node(
            package="planner",
            executable="toggle_visible"
        ),
        Node(
            package="planner",
            executable="change_comment"
        ),
        Node(
            package="planner",
            executable="toggle_navigation"
        ),
        Node(
            package="planner",
            executable="clear_state"
        ),
        Node(
            package="planner",
            executable="change_target_index"
        ),
        Node(
            package="planner",
            executable="add_pulse_sequence"
        ),
        Node(
            package="planner",
            executable="rename_pulse_sequence"
        ),
        Node(
            package="planner",
            executable="toggle_select_pulse_sequence"
        ),
        Node(
            package="planner",
            executable="remove_pulse_sequence"
        ),
        Node(
            package="planner",
            executable="remove_pulse"
        ),
        Node(
            package="planner",
            executable="set_pulse_sequence_isi"
        ),
        Node(
            package="planner",
            executable="set_pulse_sequence_intensity"
        ),
        Node(
            package="planner",
            executable="set_pulse_intensity"
        ),
        Node(
            package="planner",
            executable="set_pulse_isi"
        ),
        Node(
            package="planner",
            executable="toggle_visible_pulse"
        ),
        Node(
            package="planner",
            executable="toggle_select_pulse"
        ),
        Node(
            package="planner",
            executable="change_pulse_index"
        ),
        Node(
            package="planner",
            executable="master_state"
        ),
        Node(
            package="planner",
            executable="add_pulse_to_pulse_sequence"
        ),
        Node(
            package="planner",
            executable="set_target_orientation"
        )
    ]

    for node in nodes:
        ld.add_action(node)

    return ld
