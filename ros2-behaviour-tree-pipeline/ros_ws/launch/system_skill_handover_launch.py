from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_skill_handover',
            executable='system_skill_handover',
            name='system_skill_handover',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gesture_detector',
            executable='gesture_detector',
            name='gesture_detector',
            output='screen'
        ),
        Node(
            package='human_intent_detector',
            executable='human_intent_detector',
            name='human_intent_detector',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='robot_control',
            name='robot_control',
            output='screen'
        ),
    ])
