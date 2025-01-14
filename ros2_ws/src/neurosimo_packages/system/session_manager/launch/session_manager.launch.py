from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mtms_device_enabled_arg = DeclareLaunchArgument(
        'mtms_device_enabled',
        default_value='false',
        description='Flag to enable mTMS device session management'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for SessionManager node'
    )

    # Launch SessionManager only if MTMS_DEVICE_ENABLED is not 'true'
    launch_condition = IfCondition(
        ["'", LaunchConfiguration('mtms_device_enabled'), "' != 'true'"]
    )

    session_manager_node = Node(
        package='session_manager',
        executable='session_manager',
        name='session_manager',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=launch_condition
    )

    # Log information about whether SessionManager is launched.
    log_info = LogInfo(
        condition=launch_condition,
        msg='Launching SessionManager node.'
    )

    log_skip = LogInfo(
        condition=IfCondition(
            ["'", LaunchConfiguration('mtms_device_enabled'), "' == 'true'"]
        ),
        msg='MTMS_DEVICE_ENABLED is true. SessionManager node will not be launched.'
    )

    return LaunchDescription([
        mtms_device_enabled_arg,
        log_level_arg,
        log_info,
        log_skip,
        session_manager_node,
    ])
