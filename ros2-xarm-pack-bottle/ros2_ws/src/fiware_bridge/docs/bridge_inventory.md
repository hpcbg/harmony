# FIWARE Bridge Topic Inventory (Phase 1)

> Source of truth: `fiware_bridge/config/bridge_config.yaml` (the sample config that
> ships with the package). The demonstrator's runtime config
> (`ros2_ws/config/fiware_bridge_config.yaml`, referenced by `complete.launch.py`) is
> user-generated and may extend this set; regenerate the inventory if that file is added.
>
> Purpose: decide which topics can move to the **optional Orion-LD DDS bridge** backend
> (`bridge_backend:=dds`) vs. which must stay on the custom Python node
> (`bridge_backend:=node`, the default).

## DDS-eligibility rule

The generic Orion-LD DDS bridge surfaces a `std_msgs/String` DDS topic as the NGSI-LD
attribute path `.<attribute>.value.data` (same shape as the ARISE `chatter` hello world).
Therefore:

- **DDS-eligible = `std_msgs/String` only** (typically carrying JSON-in-string).
- Every other `std_msgs` type (`Bool`, `Int32`, …) and all `custom_interfaces/*` types are
  **not** cleanly representable through the generic bridge and stay on the custom node.
- The node-only features used by some mappings — `value_mapping` (e.g. `"ON" → true`) and
  `decode_base64` — are **not** reproduced by the generic DDS bridge, which is an additional
  reason the Bool/Int mappings remain node-only.

## DDS topic naming

ROS 2 topic `/<a>/<b>` → DDS topic **`rt/<a>/<b>`** (strip the leading `/`, prepend `rt/`).
The bridge's `domain` must equal the project's `ROS_DOMAIN_ID`. No `ROS_DOMAIN_ID` is set
anywhere in the repo, so it defaults to **`0`** — verify at runtime before relying on it.

## Inventory

| ROS 2 topic | Direction | Msg type | NGSI(-LD) entityType | entityId (NGSI-v2 → proposed NGSI-LD URN) | Attribute | DDS topic | DDS-eligible? |
|---|---|---|---|---|---|---|---|
| `/start_button` | FIWARE→sub | `std_msgs/Bool` | `Device` | `M5Stick:001` → `urn:ngsi-ld:Device:M5Stick:001` | `buttonBlue` | `rt/start_button` | **No** — Bool + `value_mapping` |
| `/stop_button` | FIWARE→sub | `std_msgs/Bool` | `Device` | `M5Stick:001` → `urn:ngsi-ld:Device:M5Stick:001` | `buttonRed` | `rt/stop_button` | **No** — Bool |
| `/angle` | FIWARE→sub | `std_msgs/Int32` | `Device` | `M5Stick:001` → `urn:ngsi-ld:Device:M5Stick:001` | `angle` | `rt/angle` | **No** — Int32 |
| `/user_inputs/voice_command` | FIWARE→sub | `std_msgs/String` | `VoiceCommand` | `VoiceCommand:operator-1` → `urn:ngsi-ld:VoiceCommand:operator-1` | `command` | `rt/user_inputs/voice_command` | **Yes** (see direction note) |
| `/user_inputs/gesture_command` | FIWARE→sub | `std_msgs/String` | `GestureDetector` | `GestureDetector:operator-1` → `urn:ngsi-ld:GestureDetector:operator-1` | `command` | `rt/user_inputs/gesture_command` | **Yes** (see direction note) |
| `/system_skill_pick_and_place/status` | pub→FIWARE | `std_msgs/String` | `Status` | `SystemSkillPickAndPlace` → `urn:ngsi-ld:Status:SystemSkillPickAndPlace` | `status` | `rt/system_skill_pick_and_place/status` | **Yes** |

## Summary

**DDS-eligible (3) — validated end-to-end against Orion-LD `1.13.0-PRE-1835`:**
- `/user_inputs/voice_command` — ✅ **works both directions.** ROS→FIWARE (`PICK` appeared on
  `urn:ngsi-ld:VoiceCommand:operator-1.command`) and FIWARE→ROS via `PATCH` (republished on the
  DDS topic) both confirmed.
- `/user_inputs/gesture_command` — ✅ **works** (structurally identical; FIWARE→ROS `PATCH` of
  `CAP_PLACED` confirmed on the ROS subscriber).
- `/system_skill_pick_and_place/status` — ⚠️ **DDS-eligible by type, but BLOCKED by its name.**
  See the reserved-`status` gotcha below. Keep it on the **node** backend, or rename its leaf.

### ⚠️ Gotcha: a topic whose leaf segment is exactly `status` does not bridge

Confirmed empirically on `fiware/orion-ld:1.13.0-PRE-1835`: a `std_msgs/String` published on a
DDS topic ending in `/status` is **never applied** to the NGSI-LD entity (it stays
`"uninitialized"`), even though DDS discovery/matching succeeds and the bridge's reader is present.
Isolation test (same namespace, type, publisher, config — only the leaf name changes):

| Leaf name | Result |
|---|---|
| `rt/foo/two`, `rt/foo/state`, `rt/foo/status2`, `rt/foo/mystatus` | ✅ updates |
| `rt/foo/status` | ❌ stays `uninitialized` |

The Orion-LD DDS module reserves the leaf `status` (it overlaps ROS 2 actions' `status` /
`action_msgs/GoalStatusArray` handling). This is why the ARISE task's own example used a
`status_json` leaf rather than `status`.

**Consequence for `/system_skill_pick_and_place/status`:** to carry this signal on the DDS path,
the publishing ROS 2 node must publish on a non-`status` leaf (e.g.
`/system_skill_pick_and_place/status_json`). Until then it remains a **node-backend-only** topic.

**Direction note:** the two `*_command` topics flow FIWARE→ROS in the demonstrator. The DDS bridge
is bidirectional — a FIWARE `PATCH` to the mapped attribute republishes on `rt/<topic>` — and this
was the path validated above.

**Not DDS-eligible (3) — stay on the custom node:**
- `/start_button` (`Bool`, also relies on `value_mapping`)
- `/stop_button` (`Bool`)
- `/angle` (`Int32`)

  These keep the `bridge_backend:=node` path mandatory whenever the M5Stack IoT inputs are in
  use. The DDS path is therefore a **partial, opt-in alternative** for the String/JSON topics, not
  a full replacement for the node.

## Notes for later phases

- `custom_interfaces/*` topics (task/arm action topics listed in the top-level README's interface
  table) are out of scope for the DDS path — they are not in `bridge_config.yaml` and are not
  `std_msgs/String`.
- The proposed NGSI-LD URNs above are suggestions for the DDS/NGSI-LD path; the existing NGSI-v2
  short IDs remain unchanged on the node path. Keep both documented so the two backends do not
  drift — Phase 3's generator should derive the DDS config from this same source mapping.
