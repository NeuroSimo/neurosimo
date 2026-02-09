from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch decider node.
    """
    log_arg = DeclareLaunchArgument(
        "log-level",
        default_value=["info"],
        description="Logging level",
    )

    logger = LaunchConfiguration("log-level")

    node = Node(
        package="decider",
        executable="decider",
        name="decider",
        arguments=['--ros-args', '--log-level', logger]
    )

    return LaunchDescription([
        log_arg,
        node
    ])
