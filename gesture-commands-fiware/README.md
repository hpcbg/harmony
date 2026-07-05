# Gesture Commands to FIWARE

> **A reusable module of the [HARMONY demonstrator](../README.md).** HARMONY combines seven
> independently reusable ARISE modules; this is one of them. **Standalone**, it is a MediaPipe
> gesture→command service — redefine the gesture set for a different application. **In the HARMONY
> demonstrator**, it sends recognised hand-gesture commands to the robot. See the
> [top-level README](../README.md) for how the modules fit together.

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

## Backends — `GESTURE_BACKEND`

The module has two interchangeable publishing backends, selected with the
`GESTURE_BACKEND` environment variable. The MediaPipe recognition is identical in
both; only the destination of a confirmed gesture differs.

| `GESTURE_BACKEND` | Path | What happens on a confirmed gesture |
|---|---|---|
| `fiware_v2` *(default)* | Gesture → Orion (NGSI-v2) | `POST`/`PATCH` `/v2/entities` on `GestureDetector:operator-1` — the integrated node-bridge demo. Unchanged behaviour. |
| `ros2` *(experimental)* | Gesture → ROS 2 → DDS → NGSI-LD | Publishes `std_msgs/String` on `/user_inputs/gesture_command`. Orion-LD's built-in DDS bridge (`-wip dds`) maps that topic to NGSI-LD. |

```bash
./run.sh                        # default = fiware_v2 (NGSI-v2)
GESTURE_BACKEND=fiware_v2 ./run.sh
GESTURE_BACKEND=ros2 ./run.sh   # DDS-native
```

For the `ros2` backend, `run.sh` auto-sources a ROS 2 environment if `rclpy`
isn't already importable — **Vulcanexus Jazzy first** (the ARISE-compliant
choice), then plain ROS 2 Jazzy as a local-debugging fallback. If neither is
present it prints the Vulcanexus guidance and exits.

### DDS-native gesture command mode (`GESTURE_BACKEND=ros2`)

- The **default mode is still FIWARE NGSI-v2** — nothing about the integrated
  demonstrator changes unless you opt in.
- In the **experimental `ros2` mode** the module does **not** call
  `/v2/entities` at all. It initialises `rclpy` and publishes each confirmed
  gesture (`NO_HAND`, `CAP_PLACED`, `SIDE_GRIP`) — values **unchanged** — as a
  `std_msgs/String` on the ROS 2 topic `/user_inputs/gesture_command`.
- **Orion-LD's DDS bridge then maps the ROS 2 topic to NGSI-LD**, landing the
  gesture on `urn:ngsi-ld:GestureDetector:operator-1`, attribute `command`
  (`command.value.data`).
- **This avoids the NGSI-v2 501 errors in DDS mode**: the `-mongocOnly` DDS
  broker rejects every NGSI-v2 request with HTTP 501, so the `ros2` backend goes
  over DDS instead of HTTP.

**No-camera self-test (`TEST_GESTURE`).** To validate the path without a camera,
OpenCV or MediaPipe, set `TEST_GESTURE`; the module waits for a matched DDS
subscriber, publishes the gesture(s), and exits:

```bash
GESTURE_BACKEND=ros2 TEST_GESTURE=SIDE_GRIP ./run.sh
GESTURE_BACKEND=ros2 TEST_GESTURE="NO_HAND SIDE_GRIP" ./run.sh
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
ros2 topic echo /user_inputs/gesture_command       # → data: SIDE_GRIP

# Terminal C — gesture, DDS-native backend
cd ../../gesture-commands-fiware
GESTURE_BACKEND=ros2 ./run.sh                       # camera mode — show side grip
GESTURE_BACKEND=ros2 TEST_GESTURE=SIDE_GRIP ./run.sh   # …or no-camera self-test

# Verify the NGSI-LD value on the DDS broker
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:GestureDetector:operator-1?local=true" \
  -H 'Accept: application/json' | jq '.command.value'    # → { "data": "SIDE_GRIP" }
```
