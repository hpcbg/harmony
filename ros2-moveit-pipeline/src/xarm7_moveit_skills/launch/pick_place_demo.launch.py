from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()
    moveit_config = MoveItConfigsBuilder("UF_ROBOT", package_name="xarm7_moveit_config").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="xarm7_moveit_skills",
        executable="pick",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])