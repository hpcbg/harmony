from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('fiware_bridge').find('fiware_bridge')

    bottle_detector_bridge = Node(
        package='bottle_detector_bridge',
        executable='bottle_detector_bridge',
        parameters=[{
                'config_path': './config/bottle_detector_bridge.json'
        }]
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
        }.items()
    )

    system_skill_pick_and_place = Node(
        package='system_skill_pick_and_place',
        executable='system_skill_pick_and_place',
        output='screen'
    )

    xarm_pick_and_place = Node(
        package='xarm_pick_and_place',
        executable='xarm_pick_and_place',
        parameters=[{
                'config_path': './config/xarm_pick_and_place.json'
        }]
    )

    return LaunchDescription([
        bottle_detector_bridge,
        fiware_bridge,
        system_skill_pick_and_place,
        xarm_pick_and_place
    ])
