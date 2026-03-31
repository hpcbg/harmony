# ROS 2 and FIWARE integration for Bottle Pack operation

This repository contains ROS 2 packages which allows integration of ROS 2 and FIWARE for execution of bottle detection and bottle pack or bottle pick and place operation with the xArm7 robot.

## Instalation

You need to install Py Trees to your ROS 2 installation: `sudo apt install ros-jazzy-py-trees`.

You can also install Vulcanexus.

## Start Procedure

You need to create the configuration files in [./ros2_ws/config/](./ros2_ws/config/). Use the provided sample config files and create similar files but without the `.tpl` extension.

For the execution on the bottle pack operation with the xArm7 robot you need to use the template in [./ros2_ws/config/xarm_pack_bottle.json.tpl](./ros2_ws/config/xarm_pack_bottle.json.tpl) and set the corresponding joint angles in degrees or space coordinates in mm. The used values are as follows:
- GRIPPER_OPEN_POS: position of the gripper when it is opened;
- GRIPPER_CLOSE_POS: position of the gripper holding a bottle;
- HOME_POS_JOINTS_DEG: coordinates of the joints in the home (request waiting) positon;
- APPROACH_HEIGHT_MM: the safe approach height over the workspace;
- AFTER_PICK_RPY_DEG: the gripper orientation in Roll-Pitch-Yaw angles (RPY) for placing the bottle in a horizontal cap down position;
- AFTER_PICK_POS_MM: the position to which the bottle must be placed cap down;
- BOTTOM_GRIP_PICK_RPY_DEG: gripper orientation for picking the bottle standing horizontally;
- BOTTOM_PICK_POS_MM: the position from which the bottle standing horizontally will be picked. It is different from AFTER_PICK_POS_MM due to the gripper dimensions;
- BOTTOM_GRIP_HEIGHT_MM: the height from which the bottle standing horizontally will be picked;
- BOTTOM_GRIP_MOVE_JOINTS_DEG: the coordinates of the joints after the horizontally standing bottle is picked and re-orientated with the cap upwards. The bottle will be standing with the cap upwards and ready to be filled;
- FILL_APPROACH_JOINTS_DEG: the coordinates of the robot for the filling approach position, the one just below the filling station;
- FILL_POS_JOINTS_DEG: the filling position in joint space coordinates;
- CAP_POS_MM: the position for the cap operation in space coordinates;
- BOTTOM_GRIP_MOVE_RPY_DEG: the gripper RPY orientation when moving the filled bottle;
- CAP_JOINT_ID: the number of the joint which will rotate the bottle during the cap operation;
- CAP_JOINT_MIN_MAX_DEG: the minimum and the maximum degrees of the joint during the cap operation.

Next, after you have build the packages you can launch the complete system with the provided launch file with the commands:

```sh
cd ros2_ws
ros2 launch ./launch/complete.launch.py
```

You can also use the provided run script in [./run.sh](./run.sh). This file will require Vulcanexus. If you do not have Vulcanexus installed you can run the ROS 2 version in [run_wo_vulcanexus.sh](./run_wo_vulcanexus.sh).

## Overview of the packages

The packages are as follows:

- `bottle_detector_bridge`: connects to the Bottle Perception API to request image processing for bottle detection and pick and place coordinates. You need to set the URL of the APIs in the config file. You can look at the sample config file [./ros2_ws/src/bottle_detector_bridge/config/bottle_detector_bridge.json](./ros2_ws/src/bottle_detector_bridge/config/bottle_detector_bridge.json). The config file is a parameter to the ROS 2 node. You can execute the node with `ros2 run bottle_detector_bridge bottle_detector_bridge --ros-args -p config_path:=/path/to/my/config.json`.
- `custom_interfaces`: contains the definition of the used actions and messages.
- `filling_station`: contains the dummy filling station node. Used for testing purposes during the development.
- `fiware_bridge`: this node connects ROS 2 and FIWARE. You need to prepare a YAML config file for the corresponding topics and entities. Take a look at the sample config file [./ros2_ws/src/fiware_bridge/config/bridge_config.yaml](./ros2_ws/src/fiware_bridge/config/bridge_config.yaml).
- `task_pack_bottle`: provides the behavior tree for the complete pack bottle task. It waits user input from the start button and then proceeds with the bottle detection, grasping from the pick and moving to the place position, and the proceeds with the fill, cap and handover operations.
- `utils_vendor`: this package provides utility python modules which are used by the other ROS 2 packages.
- `xarm_pack_bottle`: this package contains the definitions of the movements for the various operations: pick, fill, cap, andover, etc. You need to set the robot and the motion parameters in the config file. You can look at the sample config file [./ros2_ws/src/xarm_pack_bottle/config/xarm_pack_bottle.json](./ros2_ws/src/xarm_pack_bottle/config/xarm_pack_bottle.json). The config file is a parameter to the ROS 2 node. You can execute the node with `ros2 run xarm_pack_bottle xarm_pack_bottle --ros-args -p config_path:=/path/to/my/config.json`. The launch file will require the config file with name `xarm_pack_bottle.json` to be placed in [./ros2_ws/config/](./ros2_ws/config/).

  Demonstration of the execution of the pack bottle operation is shown in the following video: [https://youtu.be/xAtnlniCpGE](https://youtu.be/EWTFuBgDNAE).

  [![The Pack Bottle Task Executed by the xArm Robot](https://img.youtube.com/vi/EWTFuBgDNAE/0.jpg)](https://youtu.be/EWTFuBgDNAE)
- `edubot_pack_bottle`: this package contains an example of definitions of the movements for the various operations: pick, fill, cap, andover, etc, but for a different robot. This time no aditional configuration file is needed. This node should not be executed together with the `xarm_pack_bottle`. It is placed here for an illustration of how a different robot can be easily integrated into the system. [./ros2_ws/launch/edubot_complete.launch.py](./ros2_ws/launch/edubot_complete.launch.py) is a sample launch file.

  Demonstration of the execution of the pack bottle operation is shown in the following video: [https://youtu.be/xAtnlniCpGE](https://youtu.be/xAtnlniCpGE).

  [![The Pack Bottle Task Executed by the Education Cobot](https://img.youtube.com/vi/xAtnlniCpGE/0.jpg)](https://youtu.be/xAtnlniCpGE)

## Pick and Place Only Demo

If you want you can the older pick and place only demo with the launch file [./ros2_ws/launch/pick_and_place.launch.py](./ros2_ws/launch/pick_and_place.launch.py). This will run the `xarm_pick_and_place` node which waits for the pick and place action and controls the xArm7 robot. You need to set the robot parameters in the config file. You can look at the sample config file [./ros2_ws/src/xarm_pick_and_place/config/xarm_pick_and_place.json](./ros2_ws/src/xarm_pick_and_place/config/xarm_pick_and_place.json). The config file is a parameter to the ROS 2 node. You can execute the node with `ros2 run xarm_pick_and_place xarm_pick_and_place --ros-args -p config_path:=/path/to/my/config.json`. It also uses the node `system_skill_pick_and_place`: provides the behavior tree for the complete pick and place system skill. It waits user input from the start button and then proceeds with the bottle detection, grasping from the pick and moving to the place position.