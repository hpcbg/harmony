import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'xarm7_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Absolute paths to resources
    urdf_path = os.path.join(pkg_share, 'urdf', 'xarm7_robot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'xarm7.rviz')

    # Read URDF file content
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
