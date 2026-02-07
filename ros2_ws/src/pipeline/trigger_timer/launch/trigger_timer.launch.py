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

    maximum_timing_error_arg = DeclareLaunchArgument(
        "maximum-timing-error",
        description="Maximum timing error (in seconds)",
    )

    simulate_labjack_arg = DeclareLaunchArgument(
        "simulate-labjack",
        description="Simulate LabJack device when hardware is not available",
    )

    loopback_latency_threshold_arg = DeclareLaunchArgument(
        "loopback-latency-threshold",
        description="Maximum loopback latency, above which stimulation is prevented",
    )

    logger = LaunchConfiguration("log-level")
    maximum_timing_error = LaunchConfiguration("maximum-timing-error")
    simulate_labjack = LaunchConfiguration("simulate-labjack")
    loopback_latency_threshold = LaunchConfiguration("loopback-latency-threshold")

    node = Node(
        package="trigger_timer",
        executable="trigger_timer",
        name="trigger_timer",
        parameters=[
            {
                "maximum-timing-error": maximum_timing_error,
                "simulate-labjack": simulate_labjack,
                "loopback-latency-threshold": loopback_latency_threshold,
            }
        ],
        arguments=['--ros-args', '--log-level', logger]
    )
    ld.add_action(node)
    ld.add_action(log_arg)
    ld.add_action(maximum_timing_error_arg)
    ld.add_action(simulate_labjack_arg)
    ld.add_action(loopback_latency_threshold_arg)

    return ld
