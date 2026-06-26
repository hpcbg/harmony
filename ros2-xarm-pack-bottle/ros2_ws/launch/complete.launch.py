from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('fiware_bridge').find('fiware_bridge')

    # node (DEFAULT) -> custom fiware_bridge node; dds -> Orion-LD DDS bridge.
    bridge_backend_arg = DeclareLaunchArgument(
        'bridge_backend',
        default_value='node',
        description="ROS 2 <-> FIWARE bridge backend: 'node' (default) or 'dds'"
    )

    fiware_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'fiware_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': './config/fiware_bridge_config.yaml',
            'bridge_backend': LaunchConfiguration('bridge_backend'),
        }.items()
    )

    task_pack_bottle = Node(
        package='task_pack_bottle',
        executable='task_pack_bottle',
        output='screen'
    )

    xarm_pack_bottle = Node(
        package='xarm_pack_bottle',
        executable='xarm_pack_bottle',
        parameters=[{
                'config_path': './config/xarm_pack_bottle.json'
        }],
        output='screen'
    )

    return LaunchDescription([
        bridge_backend_arg,
        fiware_bridge,
        task_pack_bottle,
        xarm_pack_bottle
    ])
