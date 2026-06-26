# Optional DDS bridge backend (Orion-LD)

This directory provides an **optional, opt-in** alternative to the custom
`fiware_bridge` Python node: FIWARE's **Orion-LD built-in DDS bridge**. On this
path Orion-LD speaks DDS directly (`-wip dds -mongocOnly`) and maps ROS 2 topics
to NGSI-LD entities from a config file — **no custom bridge node runs at all**.

It is the ARISE-native ("Vulcanexus path") integration. The custom node remains
the **default** (`bridge_backend:=node`); nothing here changes existing behaviour
unless you explicitly opt in.

```
 bridge_backend:=node  (DEFAULT)            bridge_backend:=dds  (this directory)
 ROS 2 ──topics──▶ fiware_bridge ──▶ Orion  ROS 2 ──DDS rt/*──▶ Orion-LD (-wip dds)
                   (custom .py)                                  + context_broker_config.json
```

## Contents

| File | Purpose |
|---|---|
| `generate_config.py` | Generates `context_broker_config.json` from `../config/bridge_config.yaml` (single source of truth, so the two backends never drift). |
| `context_broker_config.json` | Generated DDS→NGSI-LD mapping for the **DDS-eligible** (`std_msgs/String`) HARMONY topics. |
| `docker-compose.dds.yml` | DDS-capable Orion-LD + MongoDB (host networking). |
| `../docs/bridge_inventory.md` | Which topics are DDS-eligible and why. |

## Scope / constraints

- **`std_msgs/String` only.** The generic DDS bridge surfaces a String topic as
  `.<attribute>.value.data`. `Bool`/`Int32`/`custom_interfaces/*` topics (e.g.
  the M5Stack buttons) are **not** representable here and stay on the node path.
- **`rt/` prefix:** ROS 2 `/foo/bar` ⇄ DDS `rt/foo/bar`.
- **`ROS_DOMAIN_ID` must equal `domain`** in the config (both default to `0`).
- Run `curl` from the **host** (the broker uses host networking).
- Do **not** run this at the same time as the default
  `fiware-analytics-docker` stack — both bind host port `1026`.
- ⚠️ **Reserved leaf name `status`:** a topic whose final segment is exactly `status`
  (e.g. `rt/system_skill_pick_and_place/status`) is **not** delivered by Orion-LD's DDS
  module — it collides with ROS 2 actions' `status`/`GoalStatusArray` handling. Use a
  different leaf (e.g. `status_json`) or keep that topic on the node backend. See
  [`../docs/bridge_inventory.md`](../docs/bridge_inventory.md).

## ROS 2 environment — use the Vulcanexus Docker image

ARISE fixes **Fast DDS** as the middleware and its examples use
`eprosima/vulcanexus:jazzy-desktop`. Running the ROS 2 side from that **container**
(rather than a host ROS install) is the cleaner, ARISE-aligned route: it guarantees the
intended Vulcanexus/Fast DDS environment and avoids polluting the host. All commands below
use it. A plain ROS 2 Jazzy install on the host works too (same RMW), but is not required.

## Validation status

Validated end-to-end on this machine against `fiware/orion-ld:1.13.0-PRE-1835`, publishing from
`eprosima/vulcanexus:jazzy-desktop`:

| Topic | ROS 2 → FIWARE | FIWARE → ROS 2 (PATCH) |
|---|---|---|
| `/user_inputs/voice_command` | ✅ `PICK` → entity | ✅ |
| `/user_inputs/gesture_command` | ✅ | ✅ `CAP_PLACED` → subscriber |
| `/system_skill_pick_and_place/status` | ❌ blocked by reserved `status` leaf (stays on node backend) | — |

---

## Hello world (no xArm-7 required)

### 1. Generate the mapping

```bash
cd ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds
python3 generate_config.py        # reads ../config/bridge_config.yaml
```

This writes `context_broker_config.json` with the 3 DDS-eligible HARMONY topics:

| DDS topic | NGSI-LD entity | attribute | DDS round-trip |
|---|---|---|---|
| `rt/user_inputs/voice_command` | `urn:ngsi-ld:VoiceCommand:operator-1` | `command` | ✅ works |
| `rt/user_inputs/gesture_command` | `urn:ngsi-ld:GestureDetector:operator-1` | `command` | ✅ works |
| `rt/system_skill_pick_and_place/status` | `urn:ngsi-ld:Status:SystemSkillPickAndPlace` | `status` | ❌ reserved `status` leaf — node backend only |

### 2. Start the DDS-capable Orion-LD

```bash
docker compose -f docker-compose.dds.yml up -d
# wait a few seconds, then check the broker is reachable (run on the HOST):
curl -s http://localhost:1026/version
```

Expected: a JSON version banner from Orion-LD.

### 3. Round-trip a real HARMONY String topic (ROS 2 → FIWARE)

Publish from a **Vulcanexus container** (Fast DDS, `ROS_DOMAIN_ID=0`, host networking).
We use `/user_inputs/voice_command` — a validated, working topic (avoid the `status` leaf):

```bash
docker run -d --rm --name dds_pub --net=host --ipc=host --privileged \
  -e ROS_DOMAIN_ID=0 eprosima/vulcanexus:jazzy-desktop \
  bash -lc "source /opt/vulcanexus/jazzy/setup.bash && \
    ros2 topic pub -r 2 /user_inputs/voice_command std_msgs/msg/String 'data: PICK'"
```

In another **host** terminal, read the NGSI-LD entity the bridge created:

```bash
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:VoiceCommand:operator-1?prettyPrint=yes&local=true" \
  -H 'Accept: application/json' | jq -r '.command.value.data'
```

Expected: `PICK` (the attribute starts as `"uninitialized"` and updates within ~2 s).
Stop the publisher with `docker stop dds_pub`.

### 4. Bidirectional (FIWARE → ROS 2)

A `PATCH` to a mapped attribute republishes on the DDS topic. Subscribe in a container:

```bash
docker run -d --rm --name dds_sub --net=host --ipc=host --privileged \
  -e ROS_DOMAIN_ID=0 eprosima/vulcanexus:jazzy-desktop \
  bash -lc "source /opt/vulcanexus/jazzy/setup.bash && \
    ros2 topic echo /user_inputs/gesture_command std_msgs/msg/String"
```

…and PATCH the entity from the host:

```bash
curl -s -X PATCH \
  "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:GestureDetector:operator-1/attrs/command" \
  -H 'Content-Type: application/json' \
  -d '{"value":{"data":"CAP_PLACED"}}'
```

Expected: `docker logs dds_sub` shows `data: CAP_PLACED`. Stop it with `docker stop dds_sub`.

### 5. Tear down

```bash
docker compose -f docker-compose.dds.yml down -v
```

---

## Appendix — upstream `chatter` baseline (sanity check)

To reproduce the canonical ARISE example before involving HARMONY topics, point
the broker at a minimal chatter mapping and run the demo talker:

```jsonc
// context_broker_config.json  (temporary, for the baseline only)
{
  "dds": { "ddsmodule": { "dds": { "domain": 0, "transport": "udp" } },
    "ngsild": { "topics": {
      "rt/chatter": { "entityType": "Robot", "entityId": "urn:ngsi-ld:robot:1", "attribute": "chatter" }
  } } }
}
```

```bash
# run the demo talker in a Vulcanexus container:
docker run -d --rm --name dds_chatter --net=host --ipc=host --privileged \
  -e ROS_DOMAIN_ID=0 eprosima/vulcanexus:jazzy-desktop \
  bash -lc "source /opt/vulcanexus/jazzy/setup.bash && ros2 run demo_nodes_cpp talker"

# on the host, watch the value increment:
while true; do
  curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:robot:1?prettyPrint=yes&local=true" \
    -H 'Accept: application/json' | jq -r '.chatter.value.data'
  sleep 1
done
# stop with: docker stop dds_chatter
```

Then re-run `python3 generate_config.py` to restore the HARMONY mapping. (This baseline was
confirmed working on this machine — `chatter.value` → `{"data":"Hello World: N"}`.)

Reference: ARISE docs — *ROS 2 to FIWARE basic app*
<https://arise-framework-documentation.readthedocs.io/en/latest/rst/getting_started/ros2_fiware_basic_app.html>
