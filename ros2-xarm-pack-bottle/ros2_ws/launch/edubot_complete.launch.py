from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('fiware_bridge').find('fiware_bridge')

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
        }.items()
    )

    task_pack_bottle = Node(
        package='task_pack_bottle',
        executable='task_pack_bottle',
        output='screen'
    )

    edubot_pack_bottle = Node(
        package='edubot_pack_bottle',
        executable='edubot_pack_bottle',
        output='screen'
    )

    return LaunchDescription([
        fiware_bridge,
        task_pack_bottle,
        edubot_pack_bottle
    ])
