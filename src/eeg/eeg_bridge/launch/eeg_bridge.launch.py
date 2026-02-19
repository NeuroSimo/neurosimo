from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch EEG bridge node.
    """
    log_arg = DeclareLaunchArgument(
        "log-level",
        description="Logging level",
    )

    logger = LaunchConfiguration("log-level")
    node = Node(
            package="eeg_bridge",
            executable="eeg_bridge",
            name="eeg_bridge",
            output="screen",
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        )

    return LaunchDescription([
        log_arg,
        node
    ])
