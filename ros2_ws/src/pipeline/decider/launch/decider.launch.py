from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    log_arg = DeclareLaunchArgument(
        "log-level",
        default_value=["info"],
        description="Logging level",
    )

    minimum_intertrial_interval_arg = DeclareLaunchArgument(
        "minimum-intertrial-interval",
        description="Minimum interval between consecutive pulses (in seconds)",
    )

    logger = LaunchConfiguration("log-level")
    minimum_intertrial_interval = LaunchConfiguration("minimum-intertrial-interval")

    # Ensure that the value of the minimum_intertrial_interval argument is a float.
    #
    # This makes it possible to pass either, e.g., "3" or "3.0" as the value of the minimum_intertrial_interval argument.
    minimum_intertrial_interval_float = PythonExpression(['float(', minimum_intertrial_interval, ')'])

    node = Node(
        package="decider",
        executable="decider",
        name="decider",
        parameters=[
            {
                "minimum-intertrial-interval": minimum_intertrial_interval_float,
            }
        ],
        arguments=['--ros-args', '--log-level', logger]
    )

    return LaunchDescription([
        log_arg,
        minimum_intertrial_interval_arg,
        node
    ])
