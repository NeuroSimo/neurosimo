from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    log_arg = DeclareLaunchArgument(
        "log-level",
        default_value=["info"],
        description="Logging level",
    )

    triggering_tolerance_arg = DeclareLaunchArgument(
        "triggering-tolerance",
        description="Tolerance when triggering (in seconds)",
    )

    logger = LaunchConfiguration("log-level")
    triggering_tolerance = LaunchConfiguration("triggering-tolerance")

    node = Node(
        package="trigger_timer",
        executable="trigger_timer",
        name="trigger_timer",
        parameters=[
            {
                "triggering-tolerance": triggering_tolerance,
            }
        ],
        arguments=['--ros-args', '--log-level', logger]
    )
    ld.add_action(node)
    ld.add_action(log_arg)
    ld.add_action(triggering_tolerance_arg)

    return ld
