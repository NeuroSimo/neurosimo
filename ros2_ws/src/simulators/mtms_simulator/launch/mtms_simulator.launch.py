from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_arg = DeclareLaunchArgument(
        name="log-level",
        description="Logging level",
        default_value="info",
    )

    channels_arg = DeclareLaunchArgument(
        name="channels",
        description="Number of mTMS channels",
        default_value="5",
    )
    max_voltage_arg = DeclareLaunchArgument(
        name="max-voltage",
        description="Maximum voltage of the coils",
        default_value="1500",
    )

    charge_rate_arg = DeclareLaunchArgument(
        "charge-rate",
        description="Charging rate of the coils in J/s",
        default_value="1500",
    )

    logger = LaunchConfiguration("log-level")

    node = Node(
        package="mtms_simulator",
        executable="mtms_simulator",
        parameters=[
            {
                "channels": LaunchConfiguration("channels"),
                "max_voltage": LaunchConfiguration("max-voltage"),
                "charge_rate": LaunchConfiguration("charge-rate"),
            }
        ],
        arguments=["--ros-args", "--log-level", logger],
    )

    return LaunchDescription(
        [
            channels_arg,
            max_voltage_arg,
            charge_rate_arg,
            log_arg,
            node,
        ]
    )
