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

    disk_warning_threshold_arg = DeclareLaunchArgument(
        "disk-warning-threshold",
        description="Warning threshold for disk space (e.g., 25GiB, 500MiB)",
    )

    disk_error_threshold_arg = DeclareLaunchArgument(
        "disk-error-threshold",
        description="Error threshold for disk space (e.g., 20GiB, 100MiB)",
    )

    logger = LaunchConfiguration("log-level")
    disk_warning_threshold = LaunchConfiguration("disk-warning-threshold")
    disk_error_threshold = LaunchConfiguration("disk-error-threshold")

    node_executables = [
        "resource_monitor",
    ]

    for node_executable in node_executables:
        node = Node(
            package="resource_monitor",
            executable=node_executable,
            parameters=[
                {
                    "disk.warning_threshold": disk_warning_threshold,
                    "disk.error_threshold": disk_error_threshold,
                }
            ],
            arguments=['--ros-args', '--log-level', logger]
        )
        ld.add_action(node)

    ld.add_action(log_arg)
    ld.add_action(disk_warning_threshold_arg)
    ld.add_action(disk_error_threshold_arg)

    return ld
