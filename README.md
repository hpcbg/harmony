# HARMONY — HRI Packaging Demonstrator

> **ARISE demonstrator built from 7 reusable modules** | MIT License | ROS 2 Jazzy / Vulcanexus | FIWARE Orion v3

The **HRI Packaging Demonstrator** (HARMONY) is a demonstrator for AI-assisted, human-directed
robotic packaging. It is not a single
program but a **composition of seven independently reusable modules** — each lives in its own folder
with its own README and can be adopted on its own — wired together here into one runnable
pack-bottle demonstrator. This README documents the demonstrator and how the modules combine; each
module's own README documents its standalone use.

**End user:** [CMYK Ingredients](https://www.cmykingredients.com/) &nbsp;·&nbsp;
**Technology provider:** [High Performance Creators (HPCBG)](https://hpc.bg/)

[![Industrial setup demonstration](./images/industrial-setup-demo.png)](./images/industrial-setup-demo.png)

## Why this demonstrator

Together with our end user **[CMYK Ingredients](https://www.cmykingredients.com/)**, we are
addressing a critical challenge in industrial automation, with a focus on the sustainable
production and packaging of healthy food and bio-pharma ingredients. CMYK's primary goal is to
**scale production efficiency** to fuel growth and competitiveness — but current technologies fall
short in the face of **high variability in package sizes and formats**. A typical order includes
**30 distinct SKUs of 1,000 units each** — far too diverse for conventional conveyor-based systems,
and the broader product portfolio adds further constraints.

Manual production and packaging, while adaptable, imposes high physical and cognitive demands on
operators — reducing efficiency, increasing error rates, and affecting product quality. This
reinforces the urgent need for **collaborative, human-augmenting robotic systems** built on modular,
human-centric automation, enabled by adaptive HRI and the ARISE all-in-one middleware.

**HARMONY** tackles CMYK's need for efficient, high-precision packaging by deploying adaptive
robotics and advanced HRI to handle product diversity, cut setup time, and ensure consistent
quality.

Flexible packaging cells are hard to automate: the task changes often, a human still needs to
direct and supervise the robot in plain, natural ways, and a plant manager needs to *see* what
the cell is doing — throughput, state, and failures — without bolting on a separate monitoring
project afterwards.

The **HARMONY demonstrator** answers both needs in one runnable system by **combining seven
reusable modules**: an **AI-driven robotic packaging capability** — a robot that detects an object
with computer vision and runs a configurable pick → fill → cap → handover sequence, directed by a
human through an IoT button, voice, or hand gestures — together with a **ready-to-run monitoring &
analytics stack** (FIWARE Context Broker + time-series history + Grafana dashboards). Every command
and state change flows through a single, declarative data layer, so the same events that drive the
robot are automatically recorded, queryable over a REST API, and visualised live.

Each module is a **shareable asset in its own right** — the ROS 2 ↔ FIWARE bridge, the AI detector,
the FIWARE analytics stack, the gesture, voice and IoT command inputs, and the React dashboard can
each be picked up and reused independently (see each module's own README). The demonstrator's value
is showing them **working together**: the packaging capability produces the data, and the analytics
layer gives that data meaning. Bundling them behind one setup gives a reviewer or an adopter a
single install, a single hardware-free demonstration, and a single coherent data model to follow.

The composition is reusable beyond bottles: swap the vision model and the YAML mapping, and the same
modules drive and monitor a different AI-assisted packaging or pick-and-place task.

---

## ARISE & HARMONY Project Context

This module is a deliverable of **[ARISE](https://arise-middleware.eu/)** — the EU project
building an open middleware that lets robotics applications interoperate across **ROS 2 /
Vulcanexus** (eProsima's ROS 2 distribution) and **FIWARE** (the IoT data, history, and
dashboarding platform), targeting TRL 6-7 demonstrations of human-robot collaboration in
industrial and service environments.

**HARMONY** (*Human-Robot Automation for Modular Operations*) is a **cascade-funded** project
under ARISE, developed by HPCBG. The HARMONY demonstrator (D3) showed the end-to-end pack-bottle
task with a physical xArm-7 robot and a suite of HRI modalities (gesture, voice, IoT button, AI
vision), all integrated through the ARISE middleware stack.

This deliverable packages that demonstrator as installable, documented, runnable software — and,
just as importantly, as **seven modules that are each reusable on their own**. The configurable
**ROS 2 ↔ FIWARE bridge** (`fiware_bridge`) is the integration backbone that ties them together: it
moves robot/skill state and human commands between ROS 2 topics and FIWARE entities through a single
YAML-configurable mapping, with no robot-specific code, which is what makes the composition portable
to a new task.

**The seven reusable modules combined into the demonstrator** (detailed in
[System Overview](#system-overview)):

| # | Reusable module | Lives in | Standalone reuse | Role in the HARMONY demonstrator |
|---|---|---|---|---|
| 1 | ROS 2 system (task orchestration + `fiware_bridge` backbone) | `ros2-xarm-pack-bottle/` | Behaviour-tree task pattern + a fully generic ROS 2 ↔ FIWARE bridge (swap the YAML) | Coordinates the pick → fill → cap → handover sequence and carries every command/state change between ROS 2 and FIWARE |
| 2 | AI object detector | `ai-bottle-detector-fiware/` | Reusable Fast R-CNN + FastAPI service; retrain weights per object | Detects bottles and returns the pick pose |
| 3 | FIWARE monitoring & analytics | `fiware-analytics-docker/` (Orion + CrateDB + QuantumLeap + Grafana) | Generic FIWARE data + history + dashboard stack | Records and visualises every cycle |
| 4 | Hand-gesture recognition | `gesture-commands-fiware/` | Reusable MediaPipe gesture→command service; retrain the gesture set | Sends gesture commands to the robot |
| 5 | Voice-command recognition | `voice-commands-fiware/` | Reusable Vosk keyword→command service; redefine the keyword set | Sends spoken commands to the robot |
| 6 | React dashboard | `react-dashboard/` | Reusable widget-based FIWARE dashboard app | Live web view of system state |
| 7 | IoT device firmware | `iot-device-firmware/` | Reusable M5Stack → FIWARE button firmware | Physical start/stop operator buttons |

---

## Quick Start

The DDS-native runtime is the default path. `setup_dds.py` is the guided Setup Assistant and
`./launch_pack_bottle_dds.sh` is the one-shot launcher for the same runtime.

**1. First-time installation** — run the Setup Assistant and choose option **2, First-time
Install**. It installs all system and Python dependencies and builds the ROS 2 workspace (see
[Installation](#installation)):

```bash
python3 setup_dds.py          # → 2) First-time Install
```

**2. Daily startup** — run the Setup Assistant and choose option **1, Daily Startup (DDS)**. It
checks readiness, prompts for the gesture camera, and launches the DDS-native runtime:

```bash
python3 setup_dds.py          # → 1) Daily Startup (DDS)
```

**3. Direct startup** — once installed, bring up the whole DDS-broker-only demo in one command:

```bash
./launch_pack_bottle_dds.sh
```

**4. Hardware-free operation** — the stub modes keep the ROS 2 and DDS interfaces fully active but
open no camera devices, so the whole path runs on a machine with no cameras attached:

```bash
AI_DETECTION_MODE=stub ./launch_pack_bottle_dds.sh                     # AI detector without a camera
GESTURE_MODE=stub ./launch_pack_bottle_dds.sh                          # gesture recognition without a camera
AI_DETECTION_MODE=stub GESTURE_MODE=stub ./launch_pack_bottle_dds.sh   # both without cameras
```

See [DDS-native runtime](#dds-native-runtime) for the gesture camera configuration and the stub
modes in detail.

---

## System Overview

The full HARMONY demonstrator is a 7-subsystem pipeline for an automated bottle pick-and-place,
fill, cap, and handover operation. The human operator interacts with the system via IoT button,
voice commands, and hand gestures. The robot responds and reports its status back through the
same FIWARE data layer.

The demonstrator was **validated at TRL 6 in a real industrial environment** — deployed in the
working space of an end user and tested with factory operators interacting with the robot for the
first time. The scenario is a *semi-automated packaging process* with a strict **human-in-the-loop**
design: although the system can act autonomously, the robot performs actions only after an explicit
operator command or clearly recognised intent, which improves safety and operator trust. This
human-centric, supervised-collaboration approach is aligned with **Industry 5.0** principles and the
ARISE objective of reusable, interoperable HRI components. Observed gains in the demonstrated setup
include reduced cycle time, lower operator workload, and improved process transparency through
live monitoring.

The high-level architecture of the HARMONY demonstrator is shown below:

[![HARMONY Demonstrator Architecture](./images/ai_packaging_architecture.png)](./images/ai_packaging_architecture.png)

The flow diagram of the system skill Pick is shown below:
[![System skill Pick](./images/pick-skill-flow.png)](./images/pick-skill-flow.png)

The full demonstrator system architecture (all 7 subsystems) is shown here:

[![System Architecture](./images/system_architecture.jpg)](./images/system_architecture.jpg)

The system combines the following **seven shareable modules** into the HARMONY demonstrator. Each is
a reusable asset in its own right — usable standalone as described in its own folder README — and
plays the role noted below when combined here.

### 1. ROS 2 System

ROS 2 coordinates the pack bottle operation. The developed ROS 2 nodes bridge between the FIWARE
platform, the AI vision system, and the xArm robot. The ROS 2 graph is shown below:

[![ROS 2 Graph](./images/rosgraph.png)](./images/rosgraph.png)

The pack bottle operation is coordinated by a node that implements its logic using the Behaviour
Tree shown in the following figure:

[![Pack Bottle Behaviour Tree](./images/pack_bottle_behaviour_tree.png)](./images/pack_bottle_behaviour_tree.png)

> **As a reusable module:** the `fiware_bridge` is a fully generic, YAML-configured ROS 2 ↔ FIWARE
> bridge with no robot-specific code, and the behaviour-tree task pattern retargets to another arm or
> task. **In this demonstrator:** it coordinates the pick → fill → cap → handover sequence and moves
> every command and state change between ROS 2 and FIWARE. See
> [ros2-xarm-pack-bottle](./ros2-xarm-pack-bottle/).

### 2. AI Bottle Detector System

The AI Bottle Detector is a stand-alone vision system built on a **Fast R-CNN** model (PyTorch).
It can be executed on a dedicated GPU machine for faster inference, and provides a REST API
(FastAPI) for managing detection jobs and accessing the live camera feed, plus a notify endpoint
to work directly with FIWARE. The model can be retrained for different bottle types; tools for
dataset generation and training are included.

Before detection, the workspace plane is calibrated using a printable sheet with ArUco markers,
which lets the detector map image coordinates to robot-frame coordinates:

[![AI Bottle Calibration](./images/ai_bottle_calibration.png)](./images/ai_bottle_calibration.png)

The detector then locates each bottle and estimates its position and orientation (using the
separate `bottle` and `cap` classes to infer pose), producing the pick pose used by the robot:

[![AI Bottle Detection](./images/ai_bottle_detection.png)](./images/ai_bottle_detection.png)

> **As a reusable module:** a standalone Fast R-CNN + FastAPI detection service with its own REST
> API — retrain the weights and it detects a different object. **In this demonstrator:** it detects
> bottles and returns the pick pose that drives the robot. See
> [ai-bottle-detector-fiware](./ai-bottle-detector-fiware/).

### 3. FIWARE Platform for Communication and Data Analytics

FIWARE is used as the central storage for data generated by all system modules. It provides a
REST API for further data access. The only data not stored in FIWARE are raw images — instead,
URLs generated by the AI vision system are stored.

All historical data is also persisted in a CrateDB database via QuantumLeap and visualised in a
Grafana dashboard. A screenshot of the analytics dashboard is shown below:

[![Analytics Dashboard](./images/analytics_dashboard.png)](./images/analytics_dashboard.png)

Because every event passes through the FIWARE data layer, per-cycle performance can be analysed
directly from the recorded history — for example, the detection and pick durations of each cycle:

[![Detection and Pick Duration](./images/detection-and-pick-duration.png)](./images/detection-and-pick-duration.png)

> **As a reusable module:** a generic FIWARE data + history + dashboard stack (Orion + QuantumLeap +
> CrateDB + Grafana) that any FIWARE application can adopt. **In this demonstrator:** it records and
> visualises every packaging cycle. See [fiware-analytics-docker](./fiware-analytics-docker/).

### 4. Hand Gesture Recognition

Hand gesture commands are recognised and sent to FIWARE using the MediaPipe hand landmark model.
Recognised gestures:

- `NO_HAND` — no hand present.
- `CAP_PLACED` — middle and index fingers pointed downwards, hand stationary.
- `SIDE_GRIP` — middle and index fingers pointed horizontally, hand stationary.

The recogniser extracts hand landmarks and classifies the gesture into a command intent that is
published to FIWARE. Like ROS4HRI, it builds on Google MediaPipe body estimation (focused on the
hand) and can be seen as a hand-gesture *extension* of the ROS4HRI perception surface.

| Cap intent | Give intent |
|---|---|
| [![Gesture — Cap intent](./images/gesture_cap_intent.png)](./images/gesture_cap_intent.png) | [![Gesture — Give intent](./images/gesture_give_intent.png)](./images/gesture_give_intent.png) |

> **As a reusable module:** a standalone MediaPipe gesture→command service — redefine the gesture
> set for a different application. **In this demonstrator:** it sends gesture commands to the robot.
> See [gesture-commands-fiware](./gesture-commands-fiware/).

### 5. Voice Commands Recognition

This module listens for predefined voice keywords using the Vosk speech recognition models. The
default keyword set is: GO, STOP, PICK, CAP, GIVE, SAFE, FAST. Recognised keywords are published
to FIWARE and consumed by the ROS 2 task orchestration. The recognition pipeline (audio input →
keyword spotting → FIWARE command) is illustrated below:

[![Voice Command Recognition Pipeline](./images/voice_command_pipeline.png)](./images/voice_command_pipeline.png)

> **As a reusable module:** a standalone Vosk keyword→command service — redefine the keyword set for
> a different application. **In this demonstrator:** it sends spoken commands to the robot. See
> [voice-commands-fiware](./voice-commands-fiware/).

### 6. React Dashboard Web Application

A fully customisable React Dashboard with widget-based pages for visualising system data and
state.

[![Dashboard Screenshot](./images/react_dashboard_screenshot.png)](./images/react_dashboard_screenshot.png)

> **As a reusable module:** a customisable, widget-based React dashboard for any FIWARE deployment.
> **In this demonstrator:** it is the live web view of system state. See
> [react-dashboard](./react-dashboard/).

### 7. IoT Devices

M5Stack IoT devices are used to send operator commands via HTTP requests to FIWARE.

> **As a reusable module:** standalone M5Stack → FIWARE button firmware for any FIWARE input device.
> **In this demonstrator:** it provides the physical start/stop operator buttons. Sample Arduino
> firmware is in [iot-device-firmware](./iot-device-firmware/).

---

## Interfaces

### ROS 2 Topics

The following topics are used by the HARMONY demonstrator and bridged via the `fiware_bridge`
module. All custom message types are defined in the `custom_interfaces` package.

| Direction | Topic | Message type | Description |
|---|---|---|---|
| Input → ROS 2 | `/user_inputs/gesture_command` | `std_msgs/String` | Gesture class from MediaPipe |
| Input → ROS 2 | `/user_inputs/voice_command` | `std_msgs/String` | Recognised keyword from Vosk |
| Input → ROS 2 | `/user_inputs/start_button` | `std_msgs/Bool` | Blue button pressed (M5Stack) |
| Input → ROS 2 | `/user_inputs/stop_button` | `std_msgs/Bool` | Red button pressed (M5Stack) |
| Perception | `/bottle_detection/command` | `std_msgs/String` | Command sent to AI detector |
| Perception | `/bottle_detection/job_json` | `std_msgs/String` | Detection result JSON (base64; **node backend only** — DDS can't decode) |
| Perception | `/bottle_detection/status_json` | `std_msgs/String` | Detector status (**DDS-native `AI_BACKEND=ros2`**) |
| Perception | `/bottle_detection/bottle_count` | `std_msgs/Int32` | Detected bottle count (**DDS-native `AI_BACKEND=ros2`**) |
| Perception | `/bottle_detection/pick_pose_json` | `std_msgs/String` | Pick pose JSON (**DDS-native `AI_BACKEND=ros2`**) |
| Perception | `/bottle_detection/result_json` | `std_msgs/String` | Full detection result JSON (**DDS-native `AI_BACKEND=ros2`**) |
| Skill | `/system_skill_pick_and_place/status_json` | `std_msgs/String` | Pick-and-place skill status (`status_json` leaf bridges on DDS) |
| Task | `/task_pack_bottle/stage` | `std_msgs/String` | Current behaviour-tree stage |
| Task | `/task_pack_bottle/status_json` | `std_msgs/String` | Overall task status (`status_json` leaf bridges on DDS) |
| Arm | `/xarm_pack_bottle/pick` | `std_msgs/String` | Arm pick action trigger |
| Arm | `/xarm_pack_bottle/fill` | `std_msgs/String` | Arm fill action trigger |
| Arm | `/xarm_pack_bottle/cap` | `std_msgs/String` | Arm cap action trigger |
| Arm | `/xarm_pack_bottle/handover` | `std_msgs/String` | Arm handover action trigger |
| Arm | `/xarm_pack_bottle/robot_status` | `std_msgs/String` | Robot status feedback |

**ROS 2 distribution:** **Vulcanexus Jazzy** (Ubuntu 24.04 Noble) is the recommended environment —
ARISE fixes Fast DDS as the middleware, and Vulcanexus is eProsima's Fast-DDS-aligned ROS 2
distribution used in the ARISE middleware stack. Because Vulcanexus is a superset of standard ROS 2
Jazzy, the package also builds and runs on a plain ROS 2 Jazzy install (`install_prerequisites.sh`
installs Vulcanexus; `run.sh` and `setup.py` prefer it and fall back to `/opt/ros/jazzy`).

### FIWARE / NGSI-v2 Entities

The bridge reads from and writes to the following NGSI-v2 entities on the Orion Context Broker.
All entities use `Fiware-Service: openiot` and `Fiware-ServicePath: /`.

| Entity ID | Type | Key Attribute | Direction | Source |
|---|---|---|---|---|
| `M5Stick:001` | `Device` | `buttonBlue`, `buttonRed`, `angle` | FIWARE → ROS 2 | IoT device |
| `VoiceCommand:operator-1` | `Command` | `command` | FIWARE → ROS 2 | voice module |
| `GestureDetector:operator-1` | `Command` | `command` | FIWARE → ROS 2 | gesture module |
| `SystemSkillPickAndPlace` | `Status` | `status` | ROS 2 → FIWARE | ROS 2 node |

**Sample NGSI-v2 payload** (entity update from bridge):

```json
{
  "status": {
    "type": "Text",
    "value": "IDLE",
    "metadata": {
      "dateModified": {
        "type": "DateTime",
        "value": "2024-05-10T12:34:56.789Z"
      }
    }
  }
}
```

The bridge polls Orion at a configurable interval (default 1 s) using `GET /v2/entities/<id>?attrs=<attr>&metadata=dateModified` and only re-publishes to ROS 2 when the `dateModified` timestamp changes — preventing redundant topic floods.

### FIWARE Bridge Backends (custom node vs. DDS enabler)

The module ships **two interchangeable bridge backends**, selected at launch with
`bridge_backend:=dds|node`. The ARISE-native **DDS enabler is the default** (`bridge_backend:=dds`):
ROS 2 topics map straight into Orion-LD as NGSI-LD entities with no bridge node. The custom node
(`bridge_backend:=node`) is the NGSI-v2 alternative for the standard Orion stack.

> ⚠️ **DDS-broker-only is bridge-only.** The DDS broker is Orion-LD with `-wip dds -mongocOnly`,
> which serves **NGSI-LD only** — it returns **HTTP 501** for every NGSI-v2 request
> (*"Non NGSI-LD requests are not supported with -mongocOnly is set"*), and `-wip dds` requires
> `-mongocOnly` (without it Orion-LD's legacy mongo driver crashes on modern MongoDB). So the
> NGSI-v2 demonstrator components — **voice, gesture, AI detector, dashboard, QuantumLeap/Crate/
> Grafana analytics** — do **not** run against the DDS broker; they need the `node` backend + a
> standard Orion stack (or an NGSI-LD migration). The DDS path also requires a **Vulcanexus** ROS
> side (a plain ROS 2 Jazzy publisher doesn't propagate the DDS type schema).

| | `bridge_backend:=node` (NGSI-v2 alternative) | `bridge_backend:=dds` (default) |
|---|---|---|
| How | Custom Python node (`configurable_fiware_bridge`) over NGSI-v2 HTTP REST | **Orion-LD built-in DDS bridge** (`-wip dds -mongocOnly`) reading `context_broker_config.json` — no custom node runs |
| API | NGSI-v2 (Orion 3.10.1) | NGSI-LD (Orion-LD) |
| Topic types | All (`String`, `Bool`, `Int32`, value-mapping, base64) | **All scalar `std_msgs`** (`String`/`Bool`/`Int32`/…) — `rt/<topic>` → `.<attr>.value.data`; no `value_mapping`/base64 |
| Infra | Standard Orion; no DDS plugin | DDS-capable Orion-LD; host networking |
| ARISE alignment | Demonstrator-proven path | ARISE "DDS enabler" interoperability path |

**Custom node** — chosen for the demonstrator because mappings are fully declarative (YAML, no
code per app), the FIWARE side needs only a standard Orion broker, and HTTP REST is portable
across LANs and Docker bridges. It carries all topic types and reproduces the node-only transforms
(`value_mapping`, base64) that the DDS enabler does not.

**DDS enabler** — the ARISE-native alternative: ROS 2 / Vulcanexus DDS topics are mapped straight
into Orion-LD as NGSI-LD entities, with no bridge node at all. It covers **all
scalar `std_msgs` topics** — the bridge maps each via DDS dynamic-type discovery as
`.<attr>.value.data` (String→JSON string, Bool→JSON bool, Int→JSON number). The node-only
transforms (`value_mapping`, base64) are **not** reproduced, so wire native values upstream (PATCH a
real boolean, not `"ON"`). Everything needed to run it — the config generator, the generated
mapping, a DDS-capable `docker-compose`, and a hardware-free hello world — lives in
[`ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/`](./ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/),
with the per-class strategy and validation in
[`fiware_bridge/docs/dds_full_integration_plan.md`](./ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/docs/dds_full_integration_plan.md).
A single YAML mapping (`bridge_config.yaml`) drives **both** backends, so they never drift.

This path was **validated end-to-end** (Orion-LD `1.13.0-PRE-1835`, publishing from the
`eprosima/vulcanexus:jazzy-desktop` container): all six bridged topics round-trip in both directions
(ROS→FIWARE and FIWARE→ROS via `PATCH`) — `voice_command`/`gesture_command` (`String`), `/angle`
(`Int32`), `/user_inputs/start_button`/`/user_inputs/stop_button` (`Bool`), and
`/system_skill_pick_and_place/status_json` (`String`). Two notes: (1) the DDS broker uses **NGSI-LD
on Orion-LD**, a separate broker from the default NGSI-v2 stack, on the same host port `1026`;
(2) ⚠️ a topic whose **leaf segment is exactly `status`** is not delivered by Orion-LD's DDS module
(it collides with ROS 2 actions' `status`/`GoalStatusArray`) — **resolved here** by renaming the ROS
leaf to `status_json` (the FIWARE attribute stays `status`, so NGSI consumers are unaffected). Use
the **Vulcanexus Docker image** for the ROS 2 side — ARISE fixes Fast DDS as the middleware, and the
container guarantees that environment without touching a host ROS install.

```bash
# default (ARISE-native DDS enabler; no node runs, Orion-LD does the bridging).
# Bridge-only: NGSI-LD broker, needs a Vulcanexus ROS side; NGSI-v2 components don't run here.
cd ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds
python3 generate_config.py
docker compose -f docker-compose.dds.yml up -d
ros2 launch fiware_bridge fiware_bridge.launch.py            # bridge_backend defaults to dds

# NGSI-v2 custom node (alternative; for the standard Orion stack + integrated demo)
ros2 launch fiware_bridge fiware_bridge.launch.py bridge_backend:=node
```

> ⚙️ **DDS enabler.** The DDS enabler backend covers **all scalar `std_msgs` types**
> (String/Bool/Int32/…), not just `String`, and the reserved-`status`-leaf topic is handled via a
> `status_json` rename — see
> [`fiware_bridge/docs/dds_full_integration_plan.md`](./ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/docs/dds_full_integration_plan.md).
>
> ✅ **Everything except the React dashboard is validated end-to-end on the DDS backend.**
> The bridge, the voice / gesture / IoT-button command inputs, the AI bottle detector (status, bottle
> count, pick pose, full result, and the real Faster-R-CNN perception path), and the
> `task_pack_bottle` behaviour tree all round-trip over **ROS 2 → DDS → Orion-LD (NGSI-LD)**, verified
> under Vulcanexus by [`./run_dds_regression_tests.sh`](#dds-native-validation). Only the **React
> dashboard** requires the custom-node / NGSI-v2 backend (it reads NGSI-v2, which the
> `-mongocOnly` DDS broker does not serve), so the full dashboard demonstrator runs over
> the custom node.

### ROS4HRI Alignment

The HARMONY demonstrator operates at the **command/intent layer** — it consumes post-processed
intent signals (gesture class string, voice keyword string, button press boolean) rather than raw
human-perception streams. The ROS4HRI standard (`hri_msgs`) defines perception-level topics
(`/humans/persons/*`, `/humans/voices/*`, `/humans/bodies/*`) which are upstream of this layer.
The table below documents the alignment and justifies the gaps:

| HRI concept | Signal in HARMONY | ROS4HRI alignment | Justification |
|---|---|---|---|
| Gesture intent | `/user_inputs/gesture_command` (std_msgs/String) | **Extension** — custom topic | No ROS4HRI gesture-intent standard exists. Signal is a classified gesture string (e.g. `CAP_PLACED`), not a raw body pose. |
| Speech intent | `/user_inputs/voice_command` (std_msgs/String) | **Extension** — custom topic | No ROS4HRI speech-intent standard exists. Signal is a recognised keyword string (e.g. `PICK`), downstream of Vosk ASR. |
| Operator state | `/user_inputs/start_button`, `/user_inputs/stop_button` (std_msgs/Bool) | **N/A** — operator control, not human perception | Start/stop buttons are control signals, not human-state observations. |
| Human presence | Not implemented in this release | **Planned** | Future integration with `hri_body_detect` (PAL Robotics) will publish ROS4HRI-compliant `/humans/bodies/*` topics. Gesture and voice pipelines can be gated on human presence once that integration is complete. |

---

## Installation

> **You do not have to install these by hand.** The Setup Assistant's **First-time Install**
> installs everything listed below for you — system packages (via `install_prerequisites.sh`),
> every module's Python dependencies (`venv` + `torch_venv`), and the built ROS 2 workspace. Run
> `python3 setup_dds.py` for the DDS-native runtime (or `python3 setup.py` for the NGSI-v2 stack)
> and choose **First-time Install**. The manual steps below (Option B) are only needed if you want
> to install piece by piece.

### Prerequisites

| Dependency | Version | Required for | Notes |
|---|---|---|---|
| Ubuntu | 24.04 (Noble) | All | Tested OS |
| Vulcanexus Jazzy (or ROS 2 Jazzy) | Jazzy | `fiware_bridge`, ROS 2 nodes | Vulcanexus recommended (Fast DDS); installed by `install_prerequisites.sh` |
| Docker + Docker Compose v2 | ≥ 24 / v2 | FIWARE analytics stack | Installed by `install_prerequisites.sh` |
| Node.js | ≥ 20.19 LTS | React dashboard (Vite 7) | Installed by `install_prerequisites.sh` (NodeSource) |
| Python | 3.10+ | All Python modules | Bundled with Ubuntu 24.04 |
| `requests` | ≥ 2.28 | `fiware_bridge` | In `requirements.txt` (`pip install -r requirements.txt`) |
| `pyyaml` | ≥ 6.0 | `fiware_bridge` | In `requirements.txt` (`pip install -r requirements.txt`) |
| Node.js / npm | ≥ 18 | React dashboard only | Installed by `install_prerequisites.sh` |
| MediaPipe | ≥ 0.10 | Gesture module only | `pip install -r gesture-commands-fiware/requirements.txt` |
| Vosk | ≥ 0.3 | Voice module only | `pip install -r voice-commands-fiware/requirements.txt` |
| xArm SDK | latest | Robot arm only | Closed-source vendor SDK; **not required in stub mode** |
| xArm-7 robot | hardware | Robot arm only | **Not required in stub mode** |
| USB camera | hardware | AI detector only | **Not required in stub mode** |
| Microphone | hardware | Voice module only | **Not required in stub mode** |

**Simulation / hardware-free path:** the `fiware_bridge` and the FIWARE analytics stack can be
fully exercised without robot hardware, a camera, or a microphone. The Quick Start's
[stub modes](#dds-native-runtime) use this path.

### Option A — Guided setup with the Setup Assistant (recommended)

The repository ships a **Setup Assistant** (`setup.py`) that automates both first-time
installation and daily startup. It is a pure-Python wizard (no extra dependencies) that:

- creates the Python virtual environments (`venv` for the gesture/voice modules and a
  separate `torch_venv` for the AI detector, since PyTorch must be installed against the
  correct CPU/CUDA wheel),
- installs each module's `requirements.txt`,
- runs `colcon build` on the ROS 2 workspace,
- generates the per-module config files, and
- on subsequent runs, checks readiness and launches the selected services in separate
  terminal tabs.

```bash
python3 setup.py
# Choose "First-time Install" on the first run, "Daily Startup" thereafter.
```

For the **DDS-native / NGSI-LD runtime**, use the DDS variant instead — its **First-time
Install** does the same full install, and its **Daily Startup (DDS)** wires the DDS-broker-only
path (and prompts for the gesture camera):

```bash
python3 setup_dds.py
# "First-time Install" on the first run, "Daily Startup (DDS)" thereafter.
```

Either Setup Assistant calls `install_prerequisites.sh` under the hood for the system-level
packages (Docker, Node.js, Vulcanexus Jazzy). You can also run that script on its own — see
Option B.

### Option B — Manual installation

**1. System packages.** Run the provided install script to install Docker, Node.js, and
Vulcanexus Jazzy (ROS 2 + Fast DDS):

```bash
chmod +x install_prerequisites.sh
./install_prerequisites.sh
```

**2. Python virtual environment.** Create and activate a venv, then install the bridge
dependencies (this is all that is needed to run the `fiware_bridge` on its own):

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt          # requests + pyyaml
```

**3. Optional HRI modules.** Install only the ones you need (the AI detector needs a
PyTorch wheel and is best handled by the Setup Assistant's dedicated `torch_venv`):

```bash
# Gesture recognition (optional)
pip install -r gesture-commands-fiware/requirements.txt

# Voice recognition (optional — also requires: sudo apt install portaudio19-dev)
pip install -r voice-commands-fiware/requirements.txt
```

**4. Build the ROS 2 workspace:**

```bash
cd ros2-xarm-pack-bottle/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Basic Demo — Full System

**Scenario:** human operator presses a button or says "GO PICK" → robot detects a bottle, picks
it up, fills it, caps it, and hands it over. The entire operation state is logged in FIWARE and
visible on the Grafana and React dashboards.

**Prerequisites:** complete hardware setup (xArm-7, camera, microphone, M5Stack button),
configuration files created as described in the per-module READMEs.

The [Quick Start](#quick-start) brings up the DDS-native runtime. `./launch_pack_bottle_dds.sh`
starts the Orion-LD DDS broker, the `ros2` voice/gesture/AI backends, and the `dds` bridge backend
(the React dashboard is omitted — it reads NGSI-v2, which the `-mongocOnly` DDS broker does not
serve). It requires a Vulcanexus ROS side, and the DDS broker binds host port 1026 (run only one
Orion at a time). The NGSI-v2 stack (standard Orion + QuantumLeap + CrateDB + Grafana + React
dashboard) starts with `./launch_pack_bottle.sh` instead.

**Operator input:** press the blue button on the M5Stack, or say "GO PICK".

**Expected output:** robot executes the pick-fill-cap-handover sequence; all stages appear in the
Grafana analytics dashboard at `http://localhost:4000` (default credentials: admin / admin).

### DDS-native runtime

By default both perception modules run in **real camera mode**: AI bottle detection uses its
camera + weights, and gesture recognition uses the camera device stored in
`gesture-commands-fiware/config/config.json` — a machine-local file (gitignored, created from
the tracked `config/config.json.tpl`):

```json
{
  "CAMERA": 0
}
```

To set the gesture camera, run `python3 setup_dds.py` and choose **Daily Startup (DDS)** — it
prompts for the camera ID (Enter keeps the current value) and writes it to `config/config.json`.
Running the gesture script directly accepts an optional `--camera N` as a one-run override, e.g.
`python gesture-commands-fiware.py --camera 2`.

**Stub modes (no camera).** Either perception module can run without a camera. Run AI bottle
detection without a camera (no weights/pipeline needed):

```bash
AI_DETECTION_MODE=stub ./launch_pack_bottle_dds.sh
```

Run gesture recognition without a camera — it idles in `NO_HAND` and needs no MediaPipe;
`TEST_GESTURE=SIDE_GRIP` (or `NO_HAND`/`CAP_PLACED`) publishes gestures headlessly:

```bash
GESTURE_MODE=stub ./launch_pack_bottle_dds.sh
```

Run both in stub mode (the whole ROS 2 → DDS → Orion-LD path with no cameras attached):

```bash
AI_DETECTION_MODE=stub \
GESTURE_MODE=stub \
./launch_pack_bottle_dds.sh
```

The stub modes keep the ROS 2 topics and the DDS → Orion-LD mappings fully active; they only skip
opening the camera devices, so the whole path runs on a machine with no cameras attached.

### Video Demonstration

Pack bottle operation with the xArm robot:

[![The Pack Bottle Task Executed by the xArm Robot](https://img.youtube.com/vi/EWTFuBgDNAE/1.jpg)](https://youtu.be/EWTFuBgDNAE)

Pack bottle operation with an educational/research cobot:

[![The Pack Bottle Task Executed by the Education Cobot](https://img.youtube.com/vi/xAtnlniCpGE/2.jpg)](https://youtu.be/xAtnlniCpGE)

Pack bottle operation in idustrial setup:

[![The Pack Bottle Task in industrial setup](https://img.youtube.com/vi/Lhe4MJ5kDoQ/2.jpg)](https://youtu.be/Lhe4MJ5kDoQ)

> **Note:** in these demonstrations the settings are tuned to **showcase the human-robot
> interaction**, not to maximise throughput. Robot speeds and dwell times are deliberately
> conservative so the multimodal HRI (voice, gesture, IoT button) is clearly visible; they are not
> indicative of the cell's achievable cycle-time performance.

### DDS-native validation

> **Scope:** on the DDS-native path, **every subsystem except the React dashboard is validated
> end-to-end on the DDS backend** — bridge, voice, gesture, IoT button, AI detector (including real
> Faster-R-CNN perception), and the `task_pack_bottle` behaviour tree. The dashboard reads NGSI-v2,
> which the `-mongocOnly` DDS broker does not serve, so the full dashboard demonstrator above
> runs over the **custom node bridge / NGSI-v2**.

The voice, gesture and AI-detector modules each provide a DDS-native
**`ros2` backend** (selected with `VOICE_BACKEND=ros2`, `GESTURE_BACKEND=ros2`, `AI_BACKEND=ros2`).
Rather than posting NGSI-v2 entities over HTTP, these publish scalar `std_msgs` topics (`String`, and
`Int32` for the bottle count); an Orion-LD broker started with `-wip dds` maps those DDS topics
directly to NGSI-LD — no custom bridge node and no NGSI-v2 `/v2` calls (so no HTTP 501 from the
`-mongocOnly` DDS broker). The AI detector exposes its result as separate DDS-native outputs
(status, bottle count, pick pose, full result JSON); processed-image URLs and base64 payloads stay
on the NGSI-v2 backend.

The end-to-end path (voice keyword, gesture state, and the AI-detector's decomposed outputs) is
validated through **ROS 2 → DDS → Orion-LD** by the top-level script:

```bash
./validate_dds_native_demo.sh
```

It checks/starts the DDS Orion-LD broker, runs each module's no-mic/no-camera self-test
(`TEST_KEYWORD` / `TEST_GESTURE` / `TEST_DETECTION_COMMAND`), confirms the message reached the ROS 2
topic, and queries the mapped NGSI-LD entity — clearly reporting whether each Orion-LD value is
**real** or still **`"uninitialized"`**:

| Module | ROS 2 topic | NGSI-LD entity (attribute) |
|---|---|---|
| Voice (`VOICE_BACKEND=ros2`) | `/user_inputs/voice_command` | `urn:ngsi-ld:VoiceCommand:operator-1` (`command`) |
| Gesture (`GESTURE_BACKEND=ros2`) | `/user_inputs/gesture_command` | `urn:ngsi-ld:GestureDetector:operator-1` (`command`) |
| AI detector (`AI_BACKEND=ros2`) | `/bottle_detection/status_json` | `urn:ngsi-ld:BottleDetectionJob:processor-01` (`status`) |
| AI detector (`AI_BACKEND=ros2`) | `/bottle_detection/bottle_count` (`Int32`) | `urn:ngsi-ld:BottleDetectionJob:processor-01` (`bottleCount`) |
| AI detector (`AI_BACKEND=ros2`) | `/bottle_detection/pick_pose_json` | `urn:ngsi-ld:BottleDetectionJob:processor-01` (`pickPose`) |
| AI detector (`AI_BACKEND=ros2`) | `/bottle_detection/result_json` | `urn:ngsi-ld:BottleDetectionJob:processor-01` (`result`) |

The script deliberately does **not** start any NGSI-v2 component (FastAPI detector service, React
dashboard, standard Orion / QuantumLeap / Crate / Grafana) and does **not** touch
`launch_pack_bottle.sh`.

> **Recommended ARISE runtime — Vulcanexus.** The FIWARE DDS Enabler only fills an NGSI-LD value
> once the publisher propagates the `std_msgs::msg::dds_::String_` TypeObject. Plain ROS 2 Jazzy
> publishes the topic (the ROS 2 topic check passes) but does not reliably propagate the type, so the
> Orion-LD value can stay `"uninitialized"`. Run the DDS-native validation under **Vulcanexus Jazzy
> (the Vulcanexus Docker image is the recommended ARISE validation runtime)** for reliable type
> propagation and real Orion-LD values. See
> [`ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/README.md`](ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds/README.md).

To run the validation inside the Vulcanexus Docker image (recommended), use the wrapper:

```bash
./run_vulcanexus_dds_validation.sh
```

It ensures the DDS Orion-LD broker is up on the host, then runs `validate_dds_native_demo.sh` inside
`eprosima/vulcanexus:jazzy-desktop` with host networking + host IPC (so DDS and Orion-LD discovery
work). Under Vulcanexus the mapped Orion-LD values come back **real** rather than `"uninitialized"`.

**Command flow into the AI detector.** The `validate_dds_native_demo.sh` checks above use each
module's self-test, which publishes outputs directly. `validate_dds_command_flow.sh` additionally
validates the real **command path** for the AI detector: it starts the detector long-lived
(`AI_BACKEND=ros2`), then updates the Orion-LD `command` attribute and confirms it flows
**Orion-LD → DDS → detector subscription → outputs** (status / bottle count / pick pose / result).
The command must be written in `std_msgs/String` DDS form (`{"value":{"data":"START"}}`); a plain
string does not propagate to DDS. The Vulcanexus wrapper takes an optional target script:

```bash
./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh
```

Run all of the above as one regression suite (DDS-native + command-flow + the
`task_pack_bottle` DDS-source / default-source / no-robot-safety checks) with
`./run_dds_regression_tests.sh` (validation only — no dashboard, no NGSI-v2 demo).

**Real perception behind the same DDS interface.** The `ros2` AI backend has two detection modes,
selected with `AI_DETECTION_MODE` — `stub` (default; no camera/PyTorch) and `real`, which runs the
**existing** detector pipeline (Faster R-CNN + ArUco homography) and publishes the real result on
**the same four topics with the same Orion-LD mappings**. Only `run_detection_stub()` is swapped for
the real call; the DDS interface, the NGSI-v2 FastAPI detector, and the dashboard are untouched. Real
mode uses the detector venv python (`torch_venv`), which imports both `rclpy` and `torch` in one
interpreter (`AI_BACKEND=ros2 AI_DETECTION_MODE=real ./run.sh`). It has been validated end-to-end —
an Orion-LD `command=START` triggering a real Faster R-CNN detection whose bottle count / pick pose /
result come back **real** in Orion-LD over DDS.

**Behaviour tree consuming DDS-native outputs.** `task_pack_bottle` already publishes its detection
trigger as `START` on `/bottle_detection/command` (accepted by both detector backends). Its result
consumer (`BottleDetectorStatus`) is source-selectable via the `TASK_DETECTOR_SOURCE` env var:
`job_json` *(default — the NGSI-v2 / node-backend `/bottle_detection/job_json` path)* or
`dds`, which consumes the DDS-native detector's atomic full result on `/bottle_detection/result_json`
(`status` / `pickPose`; the decomposed `bottle_count` / `pick_pose_json` topics carry the same data).
The detection decision (DONE + a pick pose → proceed, else retry) is identical for both:

```bash
TASK_DETECTOR_SOURCE=dds ros2 run task_pack_bottle task_pack_bottle
```

**No-robot / simulation safety.** The robot action wrapper (`RunActionAsync`) tolerates a missing
xArm action server: if the server is unavailable it logs a single warning
(`Action server <name> unavailable; skipping robot action in no-robot/simulation mode.`), the action
**fails gracefully**, and the **node stays alive** — it retries on later ticks, so a late-starting
server is picked up automatically. With the xArm servers running, behaviour is unchanged. Manual
check (no robot needed): start the node, drive a detection, and confirm the stage reaches the pick
step without a traceback:

```bash
ros2 run task_pack_bottle task_pack_bottle &        # no xArm action servers running
ros2 topic pub --once /user_inputs/start_button std_msgs/msg/Bool "{data: true}"
# (or TASK_DETECTOR_SOURCE=dds + a result_json) → stage reaches DETECT_READY / PICK,
# one "unavailable" warning is logged, and the node keeps ticking (no crash).
```

### Reuse for cylindrical objects — pick fixtures

The module is **task-agnostic**: swap the vision model and the YAML topic/entity mapping and it
drives and monitors a different AI-assisted pick-and-place task. For the demonstrated use case —
picking **cylindrical objects (bottles)** — picking accuracy and repeatability are improved by
simple **3D-printed fixtures** that constrain the object's starting pose, reducing the burden on
perception and grasp planning:

[![3D-printed pick fixtures](./images/3d-printed-fixtures.png)](./images/3d-printed-fixtures.png)

---

## Repository Structure

```
harmony/
├── README.md                          # This file — module entry point
├── LICENSE                            # MIT
├── requirements.txt                   # Python deps for the fiware_bridge module
├── setup.py                           # Setup Assistant (install wizard + daily launcher)
├── install_prerequisites.sh           # System-level install (Docker, ROS 2, Node.js)
├── launch_pack_bottle.sh              # One-shot full-system launcher
├── images/                            # Architecture diagrams, screenshots
├── ros2-xarm-pack-bottle/
│   └── ros2_ws/src/
│       ├── fiware_bridge/             # ← integration backbone (ROS 2 ↔ FIWARE bridge)
│       │   ├── fiware_bridge/configurable_fiware_bridge.py
│       │   ├── config/bridge_config.yaml   # single source of truth (both backends)
│       │   ├── launch/fiware_bridge.launch.py   # bridge_backend:=node|dds
│       │   ├── docs/bridge_inventory.md    # per-topic DDS-eligibility analysis
│       │   └── dds/                        # optional Orion-LD DDS-enabler backend
│       │       ├── generate_config.py
│       │       ├── context_broker_config.json
│       │       ├── docker-compose.dds.yml
│       │       └── README.md               # DDS hello world
│       ├── custom_interfaces/         # ROS 2 message/action definitions
│       ├── task_pack_bottle/          # Behaviour tree (demonstrator-specific)
│       ├── xarm_pack_bottle/          # xArm arm actions (demonstrator-specific)
│       └── edubot_pack_bottle/        # Edubot arm actions (demonstrator-specific)
├── ai-bottle-detector-fiware/         # Fast R-CNN + FastAPI detector (demonstrator-specific)
├── fiware-analytics-docker/           # Orion + CrateDB + QuantumLeap + Grafana stack
├── gesture-commands-fiware/           # MediaPipe gesture → FIWARE
├── voice-commands-fiware/             # Vosk ASR → FIWARE
├── react-dashboard/                   # React visualisation dashboard
└── iot-device-firmware/               # M5Stack Arduino firmware
```

**Role of each component in the demonstrator** vs. reusable module status:

| Component | Reusable as-is | Demonstrator-specific parts |
|---|---|---|
| `fiware_bridge` | Yes — any ROS 2 ↔ FIWARE use case | Config YAML is bottle/xArm-specific; swap the YAML for a new use case |
| `custom_interfaces` | Partially — `std_msgs` types only in bridge | Action types are bottle-task-specific |
| `gesture-commands-fiware` | Yes — any gesture → FIWARE integration | Gesture classes (`CAP_PLACED`, `SIDE_GRIP`) are task-specific; re-train MediaPipe labels |
| `voice-commands-fiware` | Yes — any keyword → FIWARE integration | Keyword set is bottle-task-specific; change the keyword list |
| `fiware-analytics-docker` | Yes — generic FIWARE stack | Grafana dashboard panels are bottle-task-specific |
| `ai-bottle-detector-fiware` | Partially — Fast R-CNN + FastAPI pattern is reusable | Model weights and class names are bottle-specific |
| `task_pack_bottle` | No | Entirely bottle-task-specific behaviour tree |
| `xarm_pack_bottle` / `edubot_pack_bottle` | No | xArm-7 / Edubot hardware-specific |

---

## Known Limitations

- **Hardware coupling:** the full demonstrator requires an xArm-7 or Edubot robot, a USB camera,
  an M5Stack IoT device, and a microphone. The `fiware_bridge` itself has no hardware
  dependency.
- **Polling latency:** FIWARE → ROS 2 data delivery depends on the polling interval (default 1 s).
  This is acceptable for HRI command signals but is not suitable for high-frequency sensor data.
- **DDS-broker-only is bridge-only (default backend):** the default `bridge_backend:=dds` (Orion-LD,
  NGSI-LD) covers all scalar `std_msgs` types (String/Bool/Int32/…) but the DDS broker serves
  **NGSI-LD only** — it returns HTTP 501 for NGSI-v2, so the NGSI-v2 demonstrator components
  (voice/gesture/AI/dashboard/analytics) do **not** run against it, and it needs a **Vulcanexus** ROS
  side. The `bridge_backend:=node` alternative targets NGSI-v2 (standard Orion) and carries the
  node-only transforms (`value_mapping`, base64) and `custom_interfaces/*` types — see
  [FIWARE Bridge Backends](#fiware-bridge-backends-custom-node-vs-dds-enabler).
- **Single-attribute polling:** the bridge polls individual attributes per entity. Polling
  efficiency degrades with a large number of mapped attributes.
- **Gesture classes are task-specific:** the MediaPipe gesture recogniser is trained on
  `CAP_PLACED` and `SIDE_GRIP` gestures that are specific to the bottle-cap operation. Reuse
  requires retraining or re-labelling.
- **Model weights:** the AI detector uses Fast R-CNN weights (PyTorch/torchvision) trained on
  bottle images. These weights (`*.pth`) are not committed to the repository (size); pre-trained
  bottle weights are published on Hugging Face
  ([`hpcbg/harmony-bottle-detector`](https://huggingface.co/hpcbg/harmony-bottle-detector)) and the
  setup assistant can download them. Retraining is required for other object classes; tools for
  dataset generation and training are provided.
- **Tested on Ubuntu 24.04 only:** other Linux distributions and Windows/macOS are not tested.
- **No authentication:** the FIWARE Orion instance runs without authentication in the default
  configuration. Production deployments must add Keyrock/PEP Proxy authentication.

---

## Ad-hoc Components, Proprietary Dependencies, and Future Work

**Ad-hoc / demonstrator-specific:**
- `bridge_config.yaml` contains entity IDs and topic names specific to the pack-bottle task;
  a new application must supply its own YAML.
- The behaviour tree in `task_pack_bottle` encodes the exact bottle-pick-fill-cap-handover
  sequence and is not intended for reuse.
- Robot joint angles and workspace coordinates in `xarm_pack_bottle/config/` are
  calibrated for the specific laboratory setup.

**Proprietary dependencies:**
- The xArm-7 robot requires the UFactory xArm Python SDK (open source, MIT) but communicates
  with proprietary firmware on the robot controller.
- The Edubot educational cobot uses a vendor-specific interface library.
- Neither robot SDK is required to run the `fiware_bridge` or the stub-mode demo.

**Untested configurations:**
- Multi-robot or multi-operator scenarios (single operator, single arm only tested).
- FIWARE deployments behind a reverse proxy or with HTTPS.
- Vulcanexus-specific DDS QoS profiles have not been validated; standard ROS 2 defaults are used.
- ROS 2 bag file replay as a substitute for live sensor data (planned but not tested).

**Future work:**
- ✅ **DDS backend — non-`String` topics:** `Bool`/`Int32` M5Stack inputs and the `status_json`
  topic all round-trip over DDS (see
  [`dds_full_integration_plan.md`](./ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/docs/dds_full_integration_plan.md)).
  Remaining: validate it as the *primary* backend for the full demonstrator (currently the custom
  node is the default).
- Bring NGSI-LD (`@context`-aware payloads) to the custom node backend as well, for parity with
  the DDS path.
- Integrate `hri_body_detect` (PAL Robotics) to publish ROS4HRI-compliant `/humans/bodies/*`
  topics and gate gesture/voice commands on confirmed human presence.
- Add a subscription-based (FIWARE notification) path alongside polling to reduce latency.
- Provide a Docker image for the `fiware_bridge` node to simplify deployment.

---

## Maintainer

| Field | Value |
|---|---|
| Name | Simeon Tsvetanov |
| Organisation | HPCBG |
| Email | set@hpc.bg |
| GitHub | [hpcbg/harmony](https://github.com/hpcbg/harmony) |

---

## Licensing Information

All source code in this repository is released under the **MIT License** — see [LICENSE](./LICENSE).

Third-party runtime dependencies and their licenses:

| Dependency | License | Notes |
|---|---|---|
| ROS 2 Jazzy | Apache 2.0 | System middleware |
| FIWARE Orion | AGPL-3.0 | Context Broker (self-hosted) |
| QuantumLeap | MIT | Time-series bridge |
| CrateDB | Apache 2.0 (community) | Time-series database |
| Grafana | AGPL-3.0 | Dashboard (self-hosted) |
| MediaPipe | Apache 2.0 | Gesture recognition |
| Vosk | Apache 2.0 | Speech recognition |
| PyTorch | BSD 3-Clause | AI model runtime |
| torchvision (Fast R-CNN) | BSD 3-Clause | Object detection model |
| FastAPI | MIT | REST API for AI detector |
| requests | Apache 2.0 | HTTP client in bridge |
| PyYAML | MIT | Config parsing in bridge |

The `fiware_bridge` node itself depends only on `requests` and `pyyaml` (both permissive), plus
the ROS 2 runtime (Apache 2.0). It can be shipped as a standalone MIT-licensed package without
pulling in any AGPL dependency.

---

## Repository Information

This repository is the deliverable extracted from the HARMONY demonstrator (`hpcbg/harmony-dev`).
The demonstrator history and provenance are preserved in `harmony-dev`. This repository contains
only the stable, reviewed module intended for ARISE evaluation.

When contributing using Git subtree do **not** use `--squash` so that the original commit history
is preserved.
