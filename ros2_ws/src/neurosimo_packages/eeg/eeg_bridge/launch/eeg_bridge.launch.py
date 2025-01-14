from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    log_arg = DeclareLaunchArgument(
        "log-level",
        description="Logging level",
    )

    port_arg = DeclareLaunchArgument(
        "port",
        description="Port",
    )

    num_of_tolerated_dropped_samples_arg = DeclareLaunchArgument(
        "num-of-tolerated-dropped-samples",
        description="The number of tolerated dropped samples",
    )

    eeg_device_arg = DeclareLaunchArgument(
        "eeg-device",
        description="EEG device to use: 'neurone' or 'turbolink'",
    )

    turbolink_sampling_frequency_arg = DeclareLaunchArgument(
        "turbolink-sampling-frequency",
        description="Sampling frequency of Turbolink device",
    )

    turbolink_eeg_channel_count_arg = DeclareLaunchArgument(
        "turbolink-eeg-channel-count",
        description="EEG channel count of Turbolink device",
    )

    mtms_device_enabled_arg = DeclareLaunchArgument(
        "mtms-device-enabled",
        description="Is mTMS device enabled (boolean)",
    )

    logger = LaunchConfiguration("log-level")
    node = Node(
            package="eeg_bridge",
            executable="eeg_bridge",
            name="eeg_bridge",
            parameters=[
                {
                    "port": LaunchConfiguration("port"),
                    "num-of-tolerated-dropped-samples": LaunchConfiguration("num-of-tolerated-dropped-samples"),
                    "eeg-device": LaunchConfiguration("eeg-device"),
                    "turbolink-sampling-frequency": LaunchConfiguration("turbolink-sampling-frequency"),
                    "turbolink-eeg-channel-count": LaunchConfiguration("turbolink-eeg-channel-count"),
                    "mtms-device-enabled": LaunchConfiguration("mtms-device-enabled"),
                }
            ],
            output="screen",
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', logger]
        )

    return LaunchDescription([
        log_arg,
        port_arg,
        num_of_tolerated_dropped_samples_arg,
        eeg_device_arg,
        turbolink_sampling_frequency_arg,
        turbolink_eeg_channel_count_arg,
        mtms_device_enabled_arg,
        node
    ])
