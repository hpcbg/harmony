from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "UF_ROBOT", 
        package_name="xarm7_moveit_config"
    ).to_moveit_configs()
    
    # Override the default capabilities to include MTC
    # Get the existing capabilities and add MTC capability
    existing_capabilities = moveit_config.move_group_capabilities.get("capabilities", "")
    
    if existing_capabilities:
        # Add MTC capability to existing ones
        mtc_capabilities = f"{existing_capabilities} move_group/ExecuteTaskSolutionCapability"
    else:
        # Just MTC capability
        mtc_capabilities = "move_group/ExecuteTaskSolutionCapability"

    print("Configured MoveIt! capabilities:", mtc_capabilities)
    
    # Update the moveit_config with new capabilities
    moveit_config.move_group_capabilities["capabilities"] = mtc_capabilities
    
    # Generate the standard move_group launch with modified config
    return generate_move_group_launch(moveit_config)