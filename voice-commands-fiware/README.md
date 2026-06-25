# Voice Commands to FIWARE

This repository contains a Python script which will listen to a list of predefined voice commands (keywords) and when a keyword is recognized it will be send to the FIWARE.

For the speech recognition the Vosk models are used. The default set of keywords is GO, STOP, PICK, CAP, GIVE, SAFE and FAST.

The FIWARE must be running. The voice command detection uses Fiware-Service and Fiware-Servicepath headers to scope entities into a logical tenant — the same pattern used by IoT Agents and M5Stack devices:

```
fiware_service     = "openiot"
fiware_servicepath = "/"
device_id          = "VoiceCommand:operator-1"
```

On first call, creates the entity with `POST /v2/entities`. Subsequent calls update attributes with `PATCH /v2/entities/<id>/attrs`.

## Installation

1. Install the Python dependencies: `pip install -r requirements.txt`

2. On Ubuntu you might need to install portaudio: `sudo apt install portaudio19-dev`

## Run

In order to run it you need a running FIWARE and you can run the script with `python voice-commands-fiware.py --fiware` or you can use the provided [./run.sh](./run.sh). Make sure that you are running in a correct Python environment.
