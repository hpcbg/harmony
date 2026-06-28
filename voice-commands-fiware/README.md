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

## Backends — `VOICE_BACKEND`

The module has two interchangeable publishing backends, selected with the
`VOICE_BACKEND` environment variable. The recognition (Vosk keyword spotting)
is identical in both; only the destination of a detected keyword differs.

| `VOICE_BACKEND` | Path | What happens on a detected keyword |
|---|---|---|
| `fiware_v2` *(default)* | Voice → Orion (NGSI-v2) | `POST`/`PATCH` `/v2/entities` on `VoiceCommand:operator-1` — the integrated node-bridge demo. Unchanged behaviour. |
| `ros2` *(experimental)* | Voice → ROS 2 → DDS → NGSI-LD | Publishes `std_msgs/String` on `/user_inputs/voice_command`. Orion-LD's built-in DDS bridge (`-wip dds`) maps that topic to NGSI-LD. |

```bash
./run.sh                      # default = fiware_v2 (NGSI-v2)
VOICE_BACKEND=fiware_v2 ./run.sh
VOICE_BACKEND=ros2 ./run.sh   # DDS-native
```

For the `ros2` backend, `run.sh` auto-sources a ROS 2 environment if `rclpy`
isn't already importable — **Vulcanexus Jazzy first** (the ARISE-compliant
choice), then plain ROS 2 Jazzy as a local-debugging fallback. If neither is
present it prints the Vulcanexus guidance and exits.

**No-mic self-test (`TEST_KEYWORD`).** To validate the path without a microphone
or Vosk model, set `TEST_KEYWORD`; the module waits for a matched DDS subscriber,
publishes the keyword(s), and exits:

```bash
VOICE_BACKEND=ros2 TEST_KEYWORD=PICK ./run.sh
VOICE_BACKEND=ros2 TEST_KEYWORD="PICK STOP GO" ./run.sh
```

### DDS-native voice command mode (`VOICE_BACKEND=ros2`)

- The **default mode is still FIWARE NGSI-v2** — nothing about the integrated
  demonstrator changes unless you opt in.
- In the **experimental `ros2` mode** the module does **not** call
  `/v2/entities` at all. It initialises `rclpy` and publishes each recognised
  keyword (`GO`, `STOP`, `PICK`, `CAP`, `GIVE`, `SAFE`, `FAST`) as a
  `std_msgs/String` on the ROS 2 topic `/user_inputs/voice_command`.
- **Orion-LD's DDS bridge then maps the ROS 2 topic to NGSI-LD.** With Orion-LD
  started as `-wip dds -mongocOnly` and `context_broker_config.json` mapping
  `rt/user_inputs/voice_command`, the keyword lands on
  `urn:ngsi-ld:VoiceCommand:operator-1`, attribute `command`
  (`command.value.data`).
- **This avoids the NGSI-v2 501 errors in DDS mode**: the `-mongocOnly` DDS
  broker rejects every NGSI-v2 request with HTTP 501, so the `fiware_v2` backend
  cannot talk to it. The `ros2` backend sidesteps that entirely by going over
  DDS instead of HTTP.

Before running the `ros2` backend, source a ROS 2 / Vulcanexus workspace so
`rclpy` and `std_msgs` are importable (otherwise `run.sh` prints a clear error):

```bash
source /opt/vulcanexus/jazzy/setup.bash          # or /opt/ros/jazzy/setup.bash
source ../ros2-xarm-pack-bottle/ros2_ws/install/setup.bash
VOICE_BACKEND=ros2 ./run.sh
```

> **DDS schema note.** The FIWARE DDS Enabler needs the publisher to propagate
> the `std_msgs::msg::dds_::String_` TypeObject; otherwise the attribute stays
> `"uninitialized"`. **Vulcanexus Jazzy is recommended** on the ROS side for
> reliable type propagation. See
> [`../ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/README.md`](../ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/README.md)
> for the full DDS hello-world and topic mapping.

#### Validate

```bash
# Terminal A — DDS broker
cd ../ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds
python3 generate_config.py
docker compose -f docker-compose.dds.yml up -d

# Terminal B — watch the ROS 2 topic
cd ../../../..                      # ros2_ws
source install/setup.bash
ros2 topic echo /user_inputs/voice_command       # → data: PICK

# Terminal C — voice, DDS-native backend
cd ../../voice-commands-fiware
VOICE_BACKEND=ros2 ./run.sh                        # mic mode — say "PICK"
VOICE_BACKEND=ros2 TEST_KEYWORD=PICK ./run.sh      # …or no-mic self-test

# Verify the NGSI-LD value on the DDS broker
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:VoiceCommand:operator-1?local=true" \
  -H 'Accept: application/json' | jq '.command.value'    # → { "data": "PICK" }
```
