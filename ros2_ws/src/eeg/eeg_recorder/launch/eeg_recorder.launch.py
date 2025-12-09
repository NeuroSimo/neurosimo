from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    log_arg = DeclareLaunchArgument(
        "log-level",
        description="Logging level",
    )

    logger = LaunchConfiguration("log-level")

    node = Node(
        package="eeg_recorder",
        executable="eeg_recorder",
        name="eeg_recorder",
        arguments=['--ros-args', '--log-level', logger]
    )

    ld.add_action(node)
    ld.add_action(log_arg)

    return ld
