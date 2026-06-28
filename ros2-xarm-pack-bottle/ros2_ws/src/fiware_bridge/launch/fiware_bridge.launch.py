from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('fiware_bridge').find('fiware_bridge')

    # Default config файл
    default_config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'bridge_config.yaml'
    ])

    # Bridge backend selector (DDS-broker-only architecture — dds is the default):
    #   dds (DEFAULT) -> do NOT launch the node; the Orion-LD built-in DDS bridge
    #                    does the bridging from the generated
    #                    fiware_bridge/dds/context_broker_config.json. Requires the
    #                    DDS broker (docker-compose.dds.yml, -wip dds -mongocOnly) and
    #                    a Vulcanexus ROS side (schema propagation). NOTE: the DDS
    #                    broker serves NGSI-LD only — the NGSI-v2 components
    #                    (voice/gesture/AI/dashboard/analytics) do NOT run against it.
    #   node          -> launch the custom Python fiware_bridge node (NGSI-v2) for
    #                    the standard Orion stack + the integrated demonstrator.
    bridge_backend_arg = DeclareLaunchArgument(
        'bridge_backend',
        default_value='dds',
        description="ROS 2 <-> FIWARE bridge backend: 'dds' (Orion-LD built-in DDS "
                    "bridge, default) or 'node' (custom Python node, NGSI-v2)"
    )

    use_node = IfCondition(
        PythonExpression(["'", LaunchConfiguration('bridge_backend'),
                          "' == 'node'"])
    )
    use_dds = IfCondition(
        PythonExpression(["'", LaunchConfiguration('bridge_backend'),
                          "' == 'dds'"])
    )

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
        condition=use_node,
        package='fiware_bridge',
        executable='configurable_fiware_bridge',
        name='configurable_fiware_bridge',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'fiware_host': LaunchConfiguration('fiware_host'),
            'fiware_port': LaunchConfiguration('fiware_port')
        }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')]
    )

    dds_backend_notice = LogInfo(
        condition=use_dds,
        msg="bridge_backend:=dds -> custom fiware_bridge node NOT started. "
            "Expecting the Orion-LD built-in DDS bridge to be running "
            "(see fiware_bridge/dds/: docker-compose.dds.yml + "
            "context_broker_config.json). All scalar std_msgs topics "
            "(String/Bool/Int32/...) are bridged on this path. NOTE: this "
            "replaces the standard NGSI-v2 Orion stack (same port 1026), so the "
            "NGSI-v2 components (voice/gesture/AI/dashboard) won't reach FIWARE."
    )

    return LaunchDescription([
        bridge_backend_arg,
        config_file_arg,
        log_level_arg,
        fiware_host_arg,
        fiware_port_arg,
        configurable_bridge_node,
        dds_backend_notice
    ])
