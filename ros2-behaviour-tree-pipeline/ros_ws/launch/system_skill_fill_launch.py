from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_skill_fill',
            executable='system_skill_fill',
            name='system_skill_fill',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='filling_station',
            executable='filling_station',
            name='filling_station',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='robot_control',
            name='robot_control',
            output='screen'
        ),
    ])
