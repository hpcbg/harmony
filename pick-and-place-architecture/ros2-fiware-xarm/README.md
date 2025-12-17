# ROS 2 and FIWARE integration for Bottle Pick and Place operation

This repository contains ROS 2 packages which allows integration of ROS 2 and FIWARE for execution of bottle detection and pick and place operation with the xArm7 robot.

## Start Procedure

You need to create the configuration files in [./ros2_ws/config/](./ros2_ws/config/). Use the provided sample config files and create similar files but without the `.tpl` extension.

Next, after you have build the packages you can launch the complete system with the provided launch file with the commands:

```sh
cd ros2_ws
ros2 launch ./launch/complete.launch.py
```

You can also use the provided run script in [./run.sh](./run.sh).

## Overview of the packages

The packages are as follows:

- `bottle_detector_bridge`: connects to the Bottle Perception API to request image processing for bottle detection and pick and place coordinates. You need to set the URL of the APIs in the config file. You can look at the sample config file [./ros2_ws/src/bottle_detector_bridge/config/bottle_detector_bridge.json](./ros2_ws/src/bottle_detector_bridge/config/bottle_detector_bridge.json). The config file is a parameter to the ROS 2 node. You can execute the node with `ros2 run bottle_detector_bridge bottle_detector_bridge --ros-args -p config_path:=/path/to/my/config.json`.
- `custom_interfaces`: contains the definition of the used actions and messages.
- `fiware_bridge`: this node connects ROS 2 and FIWARE. You need to prepare a YAML config file for the corresponding topics and entities. Take a look at the sample config file [./ros2_ws/src/fiware_bridge/config/bridge_config.yaml](./ros2_ws/src/fiware_bridge/config/bridge_config.yaml).
- `system_skill_pick_and_place`: provides the behavior tree for the complete pick and place system skill. It waits user input from the start button and then proceeds with the bottle detection, grasping from the pick and moving to the place position.
- `utils_vendor`: this package provides utility python modules which are used by the other ROS 2 packages.
- `xarm_pick_and_place`: this package waits for the pick and place action and controls the xArm7 robot. You need to set the robot parameters in the config file. You can look at the sample config file [./ros2_ws/src/xarm_pick_and_place/config/xarm_pick_and_place.json](./ros2_ws/src/xarm_pick_and_place/config/xarm_pick_and_place.json). The config file is a parameter to the ROS 2 node. You can execute the node with `ros2 run xarm_pick_and_place xarm_pick_and_place --ros-args -p config_path:=/path/to/my/config.json`.