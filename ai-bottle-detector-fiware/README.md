# Bottle Detection

This repository contains the complete algorithm for bottle detection.

The used AI model for bottle detection is Fast R-CNN provided by PyTorch. The camera which is used is M5Stack Timer Camera X with RTSP streaming firmware. Any other camera can also be used.

The repository contains the code needed for dataset generation and model training.

This version works only with local coordinates. The robot controller (planner) is supposed account for the offset of the marker placed at position (0, 0).

It has added support for FIWARE.

## Requirements

The code is tested on Python 3.12. The packages contained in `requirements.txt` are required: `pip install -r requirements.txt`.

Make sure that PyTorch is installed and can access the GPU correctly. You can check that everything work as expected with the Jupyter Notebook located in [./utils/bottle_detector.ipynb](./utils/bottle_detector.ipynb). You might also consider using CUDA for a faster computations.

You can check for CUDA with: `python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"`.

For older GPUs you will need older version of Python. For Ubuntu 24 it can be installed as follows:
```bash
# Add a repository for the older version of Python
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
# Install Python 3.10
sudo apt install python3.10 python3.10-venv python3.10-dev
# Create a new virtual environment
python3.10 -m venv torch_venv
# Install torch
source torch_venv/bin/activate
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
# Check the installation
python -c "import torch; print(torch.cuda.is_available())"
```

## Dataset Generation

The script in `utils/capture.py` will allow you to easily capture multiple images which will then be used for the dataset. You need to configure the camera ID or the URL of the RTSP Stream in the file `config/config.json`. You can use the `config/config.json.tpl` file for the first config file creation since the config file is not tracked by the repo.

After you capture the images you need to place them in the following dataset structure:

```bash
utils/
├── data/
│   ├── images/              # All images
│   │   ├── img001.jpg
│   │   ├── img002.jpg
│   │   └── ...
│   ├── annotations.json     # JSON with annotations
│   ├── train_annotations.json
│   ├── val_annotations.json
│   └── test_annotations.json
├── checkpoints/             # Saved models
├── train_bottle_detector.py
├── detect_bottles.py
├── annotate_dataset.py
└── requirements.txt
```

You can annotate the images with the provided annotation tool `python annotate_dataset.py --images data/images --output data/annotations.json --split`.

**Tool controls:**
- **1** - Annotate bottle
- **2** - Annotate cap
- **Click and drag** - Draw bounding box
- **R** - RESET the current image annotations (reset functionality is still under development)
- **N** - Next image (save the current annotation)
- **P** - Previous image
- **D** - Delete the last bounding box
- **S** - Save and continue
- **Q** - Save and Quit

**Important:** Always, annotate both classes if possible. You need to label two classes `bottle` and `cap` which correspond to each bottle and each bottle cap position which will be later used for detecting the bottle orientation.

After the annotation is completed move the files `train_annotations.json`, `val_annotations.json` and `test_annotations.json` to `data`.


## Model Training and Object Detection

Afer the dataset is generated you can train the model with `python train_bottle_detector.py`. You can change the `config` variable cointained into the file.

After the training is complete the best model will be saved in the `checkpoints` folder.


### Inference test

You can check the inference with the following scripts:
```bash
# Basic
python detect_bottles.py --model checkpoints/best_model.pth --image test.jpg

# Save to image
python detect_bottles.py --model checkpoints/best_model.pth --image test.jpg --output result.jpg

# Different confidence threshold
python detect_bottles.py --model checkpoints/best_model.pth --image test.jpg --confidence 0.7

# Video
python detect_bottles.py --model checkpoints/best_model.pth --video input.mp4 --output output.mp4

# Webcam stream
python detect_bottles.py --model checkpoints/best_model.pth --webcam
```

## Object Detection

You need to configure the ArUco tag numbers, locations and the pick and place locations in the file `config/config.json`. You can use the `config/config.json.tpl` file for the first config file creation since the config file is not tracked by the repo.

You can use the provided calibration sheet: [./A4_calibration_sheet.pdf](./A4_calibration_sheet.pdf). You need print it on A4 sheet and double check that the distances between the adjacent markers is 130 mm. Othewise you need to adjust the values in the configuration file. You must place the sheet in the view of the camera. The camera must on the right side of the sheet and the robot to be ot the top side ot the sheet. The XYZ axes of the robot must align with the arrows of the sheet. You need to place the robot gripper over the marker with ID 11 at the position (0, 0) and write down to the config the actual XY position and Z orientation of the robotic gripper in the `WORKAREA_POSE` field of the configuration file.

## REST API App for Bottle Detection

This repository provides a REST API App for execution of bottle detection and pick and place coordinates generation.

This is the current and maintained approach. In order to start the app you can use the [run.sh](./run.sh) script or you can run the uvicorn server by execution of the following command:

`uvicorn main:app --host 0.0.0.0 --port 22001`.

This will start the app at port `22001`.

After starting the app you can access the API documentation at the following URL: http://localhost:22001/docs

The documentation will provide you with all the information about the API and the Swager UI will allow you test and experiment with the different queries.

**IMPORTANT:** After boot up you need to perform the initial calibration by placing markers in the plane at the specified location and executing a single task. Afterwards, the markers positions will be cached and updated only if there are present again in the plane.

## Bottle Detection from FIWARE

Ypu can request a new bottle detection from the FIWARE with the following command:
```bash
curl -iX PATCH 'http://localhost:1026/v2/entities/BottleDetectionJob:processor-01/attrs'   -H 'Content-Type: application/json'   -H 'fiware-service: openiot'   -H 'fiware-servicepath: /'   -d '{"command": {"type": "Text", "value": "START"}}'
```

## Backends — `AI_BACKEND` (DDS-native, experimental)

The detector has two selectable backends, chosen with the `AI_BACKEND`
environment variable:

| `AI_BACKEND` | Path | Behaviour |
|---|---|---|
| `fiware_v2` *(default)* | NGSI-v2 FastAPI service | The full detector — `uvicorn main:app` on `:22001`, camera + PyTorch pipeline, NGSI-v2 subscription/status. **Unchanged.** |
| `ros2` *(experimental)* | ROS 2 / DDS | A minimal node (`ros2_backend.py`): subscribes `/bottle_detection/command` (accepts `START`) and publishes the detector result, **decomposed into scalar `std_msgs` topics** (status / bottle count / pick pose / full result). No `/v2` calls. |

```bash
./run.sh                          # default = fiware_v2 (full NGSI-v2 service)
AI_BACKEND=ros2 ./run.sh          # DDS-native
```

The `ros2` backend is built **incrementally**: the detector result is migrated to
DDS one output at a time so the refactor stays controlled and never breaks the
NGSI-v2 dashboard demo. In `ros2` mode the node does **not** import FastAPI, OpenCV,
PyTorch, the detection pipeline, the model weights, or the NGSI-v2 client. It
currently publishes:

| ROS 2 topic | Type | Example |
|---|---|---|
| `/bottle_detection/status_json` | `std_msgs/String` | `{"status": "PROCESSING"}` → `{"status": "DONE", "bottleCount": 1}` |
| `/bottle_detection/bottle_count` | `std_msgs/Int32` | `1` |
| `/bottle_detection/pick_pose_json` | `std_msgs/String` | `{"x": 120.0, "y": -45.0, "rotation": 30.0}` |
| `/bottle_detection/result_json` | `std_msgs/String` | `{"status": "DONE", "bottleCount": 1, "pickPose": {…}}` |

Real frame capture / inference still produce **stub values** here, and the
processed-image URLs and base64 payloads are **not migrated** — they stay on the
`fiware_v2` backend. The NGSI-v2 path's decomposed `pickX`/`pickY`/`pickRotation`
floats are unchanged; the DDS path carries the pose as one JSON string (`pickPose`).
The existing NGSI-v2 service behaviour is preserved exactly.

`run.sh` auto-sources a ROS 2 environment for the `ros2` backend if `rclpy` isn't
already importable — **Vulcanexus Jazzy first** (ARISE-compliant), then plain ROS 2
Jazzy as a local-debugging fallback.

**No-camera self-test (`TEST_DETECTION_COMMAND`).** Injects a command without a
camera, GPU, PyTorch, model weights or FIWARE NGSI-v2:

```bash
AI_BACKEND=ros2 TEST_DETECTION_COMMAND=START ./run.sh
```

### DDS mapping

With Orion-LD started as `-wip dds -mongocOnly`, each output topic is mapped to an
attribute of the same NGSI-LD entity via `context_broker_config.json`:

```
rt/bottle_detection/status_json     →  urn:ngsi-ld:BottleDetectionJob:processor-01.status
rt/bottle_detection/bottle_count    →  urn:ngsi-ld:BottleDetectionJob:processor-01.bottleCount
rt/bottle_detection/pick_pose_json  →  urn:ngsi-ld:BottleDetectionJob:processor-01.pickPose
rt/bottle_detection/result_json     →  urn:ngsi-ld:BottleDetectionJob:processor-01.result
```

`bottle_count` is an `std_msgs/Int32`, so the DDS bridge maps it to a JSON number;
the String topics map to JSON strings. These mappings are generated from the bridge YAML
(`../ros2-xarm-pack-bottle/ros2_ws/config/fiware_bridge_config.yaml`,
`ros_to_fiware`) by `dds/generate_config.py`. The `ros2` backend never calls
`/v2/entities`, so the `-mongocOnly` DDS broker never returns HTTP 501 to it.

> **DDS schema note.** The FIWARE DDS Enabler needs the publisher to propagate the
> `std_msgs::msg::dds_::String_` TypeObject; otherwise the attribute stays
> `"uninitialized"`. **Vulcanexus Jazzy is required** on the ROS side for that —
> plain ROS 2 Jazzy publishes the topic (visible via `ros2 topic echo`) but does
> not propagate the type, so the mapped NGSI-LD value will not initialise. See
> `../ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/README.md`.

#### Validate

```bash
# Terminal A — DDS broker (regenerate after editing the YAML mapping)
cd ../ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds
python3 generate_config.py --config ../../../config/fiware_bridge_config.yaml
docker compose -f docker-compose.dds.yml up -d --force-recreate orion

# Terminal B — watch the ROS 2 status topic
cd ../../../..                      # ros2_ws
source install/setup.bash
ros2 topic echo /bottle_detection/status_json     # → data: '{"status": "DONE", ...}'

# Terminal C — AI detector, DDS-native backend (no camera)
cd ../../ai-bottle-detector-fiware
AI_BACKEND=ros2 TEST_DETECTION_COMMAND=START ./run.sh

# Inspect the mapped entity (value initialises only with a Vulcanexus publisher)
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:BottleDetectionJob:processor-01?local=true" \
  -H 'Accept: application/json' | jq '.status'
```

### Command flow (Orion-LD → DDS → detector → outputs)

The `command` topic is **inbound** for the detector: Orion-LD is the DDS *writer*,
so updating the entity's `command` attribute publishes a `std_msgs/String` on
`rt/bottle_detection/command`, which the running detector (spin mode) receives and
acts on (`START` → runs detection → emits status/count/pose/result). This is the
same trigger the dashboard uses, exercised over DDS instead of NGSI-v2.

The command value must be written in `std_msgs/String` DDS form — the value is the
struct member, `{"value":{"data":"START"}}`. A plain string value does **not**
propagate to DDS:

```bash
# detector running long-lived (real subscription, no TEST_* shortcut)
AI_BACKEND=ros2 ./run.sh &

# trigger START from Orion-LD (NGSI-LD)
curl -s -X PATCH \
  "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:BottleDetectionJob:processor-01/attrs/command" \
  -H 'Content-Type: application/json' \
  -d '{"type":"Property","value":{"data":"START"}}'
```

The whole round-trip is validated by the top-level `validate_dds_command_flow.sh`
(run it under Vulcanexus with
`./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh`): it starts the
detector, injects `START` from Orion-LD, and confirms every output topic reacted
and the mapped NGSI-LD values are real.

## Other Utilities

There are several helper scripts in the `utils` folder:

- `bottle_detector.ipynb` demonstrates the object detection and recognition in a Jyputer Notebook.
- `json_config.py` loads JSON configuration file.
- `manual_detect.py` demonstrates the coordinates calculation with the help of AruCo tags.
- `pick-station-cad` is a folder containing OpenSCAD and STLs useful for a 3D printed bottle despencer and place stations.
