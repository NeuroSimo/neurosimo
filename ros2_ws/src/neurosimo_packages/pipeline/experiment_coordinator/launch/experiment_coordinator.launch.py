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

    dropped_sample_threshold_arg = DeclareLaunchArgument(
        "dropped-sample-threshold",
        description="Number of dropped samples in a second before entering error state",
    )

    logger = LaunchConfiguration("log-level")
    dropped_sample_threshold = LaunchConfiguration("dropped-sample-threshold")

    node = Node(
        package="experiment_coordinator",
        executable="experiment_coordinator",
        name="experiment_coordinator",
        parameters=[
            {"dropped-sample-threshold": dropped_sample_threshold},
        ],
        arguments=['--ros-args', '--log-level', logger]
    )
    ld.add_action(node)
    ld.add_action(log_arg)
    ld.add_action(dropped_sample_threshold_arg)

    return ld

