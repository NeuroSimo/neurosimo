from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch trigger simulator node.
    """
    ld = LaunchDescription()

    log_arg = DeclareLaunchArgument(
        "log-level",
        default_value=["info"],
        description="Logging level",
    )

    logger = LaunchConfiguration("log-level")

    node = Node(
        package="trigger_simulator",
        executable="trigger_simulator",
        name="trigger_simulator",
        namespace="neurosimo",
        arguments=['--ros-args', '--log-level', logger],
    )

    ld.add_action(node)
    ld.add_action(log_arg)
    return ld
