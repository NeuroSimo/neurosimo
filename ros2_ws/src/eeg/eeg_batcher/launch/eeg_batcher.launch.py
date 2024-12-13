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

    batch_size_arg = DeclareLaunchArgument(
        "batch-size",
        default_value=[""],
        description="How many samples to include in a batch",
    )

    downsample_ratio_arg = DeclareLaunchArgument(
        "downsample-ratio",
        default_value=[""],
        description="How much to downsample",
    )

    logger = LaunchConfiguration("log-level")

    node = Node(
        package="eeg_batcher",
        executable="eeg_batcher",
        name="eeg_batcher",
        parameters=[
            {
                "batch_size": LaunchConfiguration("batch-size"),
                "downsmaple_ratio": LaunchConfiguration("downsample-ratio"),
            }
        ],
        arguments=['--ros-args', '--log-level', logger]
    )
    ld.add_action(node)
    ld.add_action(log_arg)
    ld.add_action(batch_size_arg)
    ld.add_action(downsample_ratio_arg)

    return ld
