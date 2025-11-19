# Save this file as: ~/harmony_ws/src/xarm7_moveit_config/launch/xarm7_isaac_sim.launch.py

import os
  
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from moveit_configs_utils import MoveItConfigsBuilder

def generate_isaac_demo_launch(moveit_config, launch_package_path=None):
     """
     Launches a self contained demo
  
     launch_package_path is optional to use different launch and config packages
  
     Includes
      * static_virtual_joint_tfs
      * move_group
      * moveit_rviz
      * warehouse_db (optional)
      * ros2_control_node + controller spawners
     """

     # Command-line arguments
     ros2_control_hardware_type = DeclareLaunchArgument(
         "ros2_control_hardware_type",
         default_value="isaac",
         description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
     )

     # Declare use_sim_time argument
     use_sim_time = DeclareLaunchArgument(
         "use_sim_time",
         default_value="true",
         description="Use simulation clock if true",
     )

     if launch_package_path == None:
         launch_package_path = moveit_config.package_path
  
     ld = LaunchDescription()
     # Add launch arguments to ld
     ld.add_action(ros2_control_hardware_type)
     ld.add_action(use_sim_time)

     ld.add_action(
         DeclareBooleanLaunchArg(
             "db",
             default_value=False,
             description="By default, we do not start a database (it can be large)",
         )
     )
     ld.add_action(
         DeclareBooleanLaunchArg(
             "debug",
             default_value=False,
             description="By default, we are not in debug mode",
         )
     )
     ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
     # If there are virtual joints, broadcast static tf by including virtual_joints launch
     virtual_joints_launch = (
         launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
     )
  
     if virtual_joints_launch.exists():
         ld.add_action(
             IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(str(virtual_joints_launch)),
             )
         )

     # Given the published joint states, publish tf for the robot links
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/rsp.launch.py")
             ),
         )
     )
  
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 # str(launch_package_path / "launch/move_group.launch.py")
                 str(launch_package_path / "launch/move_group_mtc.launch.py")
             ),
         )
     )
  
     # Run Rviz and load the default config to see the state of the move_group node
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/moveit_rviz.launch.py")
             ),
             condition=IfCondition(LaunchConfiguration("use_rviz")),
         )
     )
  
     # If database loading was enabled, start mongodb as well
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/warehouse_db.launch.py")
             ),
             condition=IfCondition(LaunchConfiguration("db")),
         )
     )
  
     # Fake joint driver
     ld.add_action(
         Node(
             package="controller_manager",
             executable="ros2_control_node",
             parameters=[
                 moveit_config.robot_description,
                 str(moveit_config.package_path / "config/ros2_controllers.yaml"),
             ],
         )
     )
  
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/spawn_controllers.launch.py")
             ),
         )
     )

     # Static TF
     world2robot_tf_node = Node(
         package="tf2_ros",
         executable="static_transform_publisher",
         name="static_transform_publisher_world_to_robot",
         output="log",
         arguments=[
            "-1.05589",  # ROS X = Isaac X
            "-0.01194",  # ROS Y = -Isaac Z
            "0.99405",  # ROS Z = Isaac Y
            "0.0",       # Yaw
            "0.0",       # Pitch
            "0.0", # Roll (-90 degrees)
            "World",     # Parent frame (Isaac Sim's name)
            "world"      # Child frame (ROS 2's name, from your URDF)
         ],
         parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
     )

     # Add static tf to ld
     ld.add_action(world2robot_tf_node)
  
     return ld

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("UF_ROBOT", package_name="xarm7_moveit_config").to_moveit_configs()
    return generate_isaac_demo_launch(moveit_config)