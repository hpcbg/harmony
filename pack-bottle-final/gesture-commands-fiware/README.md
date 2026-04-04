# Gesture Commands to FIWARE

This repository contains a Python script which will recognize hand gestures commands and when a command is recognized it will be send to the FIWARE.

For the recognition the MediaPipe hand landmark model is used.

The script can recognize the following gestures:

- NO_HAND: when the hand is not present.
- CAP_PLACED: when the bottle cap is placed on top of a bottle. Middle and index fingers are pointed downwards and the hand is stationary.
- SIDE_GRIP: when the ottle is holded from the side. Middle and index fingers are pointed horizontally and the hand is stationary.

The FIWARE must be running. The gesture command detection uses Fiware-Service and Fiware-Servicepath headers to scope entities into a logical tenant — the same pattern used by IoT Agents and M5Stack devices:

```
fiware_service     = "openiot"
fiware_servicepath = "/"
device_id          = "GestureDetector:operator-1"
```

On first call, creates the entity with `POST /v2/entities`. Subsequent calls update attributes with `PATCH /v2/entities/<id>/attrs`.

## Installation

Install the Python dependencies: `pip install -r requirements.txt`


## Run

In order to run it you need a running FIWARE and you can run the script with `python gesture-commands-fiware.py --no-gui` or you can use the provided [./run.sh](./run.sh). Make sure that you are running in a correct Python environment.

You can also run it in GUI mode with: `python gesture-commands-fiware.py` or with [./run-with-gui.sh](./run-with-gui.sh).
