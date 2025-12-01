# Bottle Detection with Robotic Pick and Place

This repository contains the complete algortihm for bottle pick and place operation.

The used robot is xArm7 and the AI model for bottle detection is YOLO Oriented Bounding Boxes. The camera which is used is M5Stack Timer Camera X with RTSP streaming firmware. Any other camera can also be used.

The repository contains the code needed for dataset generation and model training as well as pick and place algorithm and also a simple xArm7 emulator of the XArmAPI which can be used for testing purposes without real robot.

## Requirements

The code is tested on Python 3.12. The following packages are required:

- ultralytics
- opencv-contrib-python
- jupyter
- xarm-python-sdk

## Dataset Generation

The script `capture.py` will allow you to easily capture multiple images which will then be used for the dataset.

After you capture the images you need to label them by using a tool like Roboflow, which supports the YOLO OBB format (Object Detection, OBB/Rotation). You need to label two classes `bottle` and `cap` which correspond to each bottle and each bottle cap position which will be later used for detecting the bottle orientation.

You need to configure the URL of the RTSP Stream in the file `config/detect_obb.json`. You can use the `config/detect_obb.json.tpl` file for the first config file creation since the config file is not tracked by the repo.

## Model Training

Afer the dataset is generated you can train the model with `train_obb.py`.

## Object Detection

For the object detection you need to use the `detect_obb.py` script.

You need to configure the ArUco tag numbers, locations and the pick and place locations in the file `config/detect_obb.json`. You can use the `config/detect_obb.json.tpl` file for the first config file creation since the config file is not tracked by the repo.

## Robotic Pick and Place

For the execution of pick and place you need to run the `xarm_pick_and_place.py` together with `detect_obb.py`. When you click on the bottle, the pick and place command will be send to the robot.

In the `config/xarm_pick_and_place.json` you need to set the IP address of the robot and whether to run the script in XArmAPI emulation mode. You can use the `config/xarm_pick_and_place.json.tpl` file for the first config file creation since the config file is not tracked by the repo.

## Utilities

There are several helper scripts in the `utils` folder:

- `bottle_detector.ipynb` demonstrates the object detection and recognition in a Jyputer Notebook.
- `json_config.py` loads JSON configuration file.
- `manual_detect.py` demonstrates the coordinates calculation with the help of AruCo tags.
- `pick-station-cad` is a folder containing OpenSCAD and STLs useful for a 3D printed bottle despencer and place stations.
- `xarm_emulator.py` is the XArmAPI emulator.
