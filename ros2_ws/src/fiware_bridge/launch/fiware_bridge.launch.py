from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('fiware_bridge').find('fiware_bridge')

    # Default config файл
    default_config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'bridge_config.yaml'
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to FIWARE bridge configuration file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    fiware_host_arg = DeclareLaunchArgument(
        'fiware_host',
        default_value='',
        description='FIWARE Orion host (overrides config file if set)'
    )

    fiware_port_arg = DeclareLaunchArgument(
        'fiware_port',
        default_value='0',
        description='FIWARE Orion port (overrides config file if set)'
    )

    configurable_bridge_node = Node(
        package='fiware_bridge',
        executable='configurable_bridge',
        name='configurable_bridge_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'fiware_host': LaunchConfiguration('fiware_host'),
            'fiware_port': LaunchConfiguration('fiware_port')
        }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        fiware_host_arg,
        fiware_port_arg,
        configurable_bridge_node
    ])
