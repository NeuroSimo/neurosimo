from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch trigger timer node.
    """
    ld = LaunchDescription()

    log_arg = DeclareLaunchArgument(
        "log-level",
        default_value=["info"],
        description="Logging level",
    )

    use_mock_labjack_arg = DeclareLaunchArgument(
        "use_mock_labjack",
        default_value=["false"],
        description="Use mock LabJack manager",
    )

    logger = LaunchConfiguration("log-level")
    use_mock_labjack = LaunchConfiguration("use_mock_labjack")

    node = Node(
        package="trigger_timer",
        executable="trigger_timer",
        name="trigger_timer",
        namespace="neurosimo",
        arguments=['--ros-args', '--log-level', logger],
        parameters=[{'use_mock_labjack': use_mock_labjack}]
    )

    ld.add_action(node)
    ld.add_action(log_arg)
    ld.add_action(use_mock_labjack_arg)
    return ld
