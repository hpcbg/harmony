# Bottle Detection

This repository contains the complete algortihm for bottle detection.

The used AI model for bottle detection is Fast R-CNN provided by PyTorch. The camera which is used is M5Stack Timer Camera X with RTSP streaming firmware. Any other camera can also be used.

The repository contains the code needed for dataset generation and model training.

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

## Other Utilities

There are several helper scripts in the `utils` folder:

- `bottle_detector.ipynb` demonstrates the object detection and recognition in a Jyputer Notebook.
- `json_config.py` loads JSON configuration file.
- `manual_detect.py` demonstrates the coordinates calculation with the help of AruCo tags.
- `pick-station-cad` is a folder containing OpenSCAD and STLs useful for a 3D printed bottle despencer and place stations.
