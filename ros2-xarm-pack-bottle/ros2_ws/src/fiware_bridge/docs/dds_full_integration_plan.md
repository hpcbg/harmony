# DDS Full-Integration — Inventory & Type-Mapping Strategy

> **Branch:** `dds-full-integration` (exploratory). **NOT the D4 deliverable.**
> `shareable-modules` is frozen and keeps the conservative, validated *String-only*
> DDS scope documented in [`bridge_inventory.md`](bridge_inventory.md). This file is the
> forward-looking plan for moving **every eligible topic** — including non-`String`
> types — onto the Orion-LD built-in DDS bridge.
>
> **Phase 1 = inventory + strategy only. No node code changes in this phase.**
> Implementation proceeds one *class* at a time, each validated on the DDS hello world
> before the next is started.

## Scope of "the node backend"

The custom node (`configurable_fiware_bridge.py`) only bridges what is declared in
`fiware_bridge/config/bridge_config.yaml` (verified: the `build/`+`install/` copies are
generated, and no runtime config extends it). That is **6 topics**. The wider arm/task
action topics in the top-level README interface table are *internal ROS 2 topics* — they
are not bridged to FIWARE today, so they are "future expansion" for the DDS path, not
part of the current node backend. They are covered under Class D below for completeness.

---

## Inventory — every node-backend topic, classified

| # | ROS 2 topic | Dir | Msg type | NGSI-LD entity (URN) | Attr | DDS topic | Class | DDS status today |
|---|---|---|---|---|---|---|---|---|
| 1 | `/user_inputs/voice_command` | FIWARE→sub | `std_msgs/String` | `urn:ngsi-ld:VoiceCommand:operator-1` | `command` | `rt/user_inputs/voice_command` | **A — String** | ✅ validated |
| 2 | `/user_inputs/gesture_command` | FIWARE→sub | `std_msgs/String` | `urn:ngsi-ld:GestureDetector:operator-1` | `command` | `rt/user_inputs/gesture_command` | **A — String** | ✅ validated |
| 3 | `/system_skill_pick_and_place/status` | pub→FIWARE | `std_msgs/String` | `urn:ngsi-ld:Status:SystemSkillPickAndPlace` | `status` | `rt/system_skill_pick_and_place/status` | **C — status-leaf collision** | ❌ blocked (reserved `status` leaf) |
| 4 | `/start_button` | FIWARE→sub | `std_msgs/Bool` | `urn:ngsi-ld:Device:M5Stick:001` | `buttonBlue` | `rt/start_button` | **B — non-String std_msgs** | ❌ untested type + `value_mapping` |
| 5 | `/stop_button` | FIWARE→sub | `std_msgs/Bool` | `urn:ngsi-ld:Device:M5Stick:001` | `buttonRed` | `rt/stop_button` | **B — non-String std_msgs** | ❌ untested type |
| 6 | `/angle` | FIWARE→sub | `std_msgs/Int32` | `urn:ngsi-ld:Device:M5Stick:001` | `angle` | `rt/angle` | **B — non-String std_msgs** | ❌ untested type |
| — | arm/task action topics (`/xarm_pack_bottle/*`, `/task_pack_bottle/*`, `Move.action`) | internal | `custom_interfaces/*`, action | — | — | `rt/…`, `rq/…`, `rr/…` | **D — custom_interfaces** | not bridged; design-only |

**Class tally:** A = 2 (done), B = 3, C = 1, D = (future / not currently bridged).

---

## How the Orion-LD DDS bridge actually maps a type

The generic bridge introspects the DDS topic's type from **DDS type discovery**
(complete `TypeObject`/XTypes propagated by the RMW) and mirrors each IDL member as a
sub-key of the NGSI-LD attribute value. For `std_msgs/*` the single member is `data`,
so every `std_msgs` topic surfaces as `<attribute>.value.data` and **only the JSON
scalar type of `data` changes with the ROS type**:

| ROS type | IDL member | Expected `<attr>.value.data` |
|---|---|---|
| `std_msgs/String` | `string data` | `"PICK"` (string) — **confirmed** |
| `std_msgs/Bool` | `boolean data` | `true` / `false` (JSON bool) — **to verify** |
| `std_msgs/Int32` | `long data` | `42` (JSON number) — **to verify** |
| custom msg of primitives | `float64 x; …` | `{ "x": …, "y": … }` (member-per-key) — **to verify** |

This is the central hypothesis: **Bool/Int32 likely need no new bridge code** — the
mapping is structural and type-generic; only empirical confirmation that the Orion-LD
DDS module accepts those primitive member kinds is missing. What the DDS path *cannot*
reproduce are the **node-only transforms**: `value_mapping` and `decode_base64`.

---

## Per-class type-mapping strategy

### Class A — `std_msgs/String` (topics 1–2) — DONE
No change. The generator already emits these; round-trip validated both directions.

### Class B — non-`String` `std_msgs` (topics 4–6) — `Bool`, `Int32`
**Strategy: rely on DDS dynamic-type introspection; move the node-only transforms out
of the bridge.**

1. **Generator change:** extend `generate_config.py` so it no longer hard-filters on
   `std_msgs/String`. Emit any `std_msgs/<Scalar>` topic (`String|Bool|Int32|Float*|…`).
   The `context_broker_config.json` entry shape is identical (`entityType`/`entityId`/
   `attribute`) — the bridge derives the member type from DDS, not from our config.
2. **Validate the type natively first** (hello world, before touching real topics):
   publish `std_msgs/Bool` and `std_msgs/Int32` on a throwaway DDS topic and confirm
   `<attr>.value.data` becomes a JSON bool / number, and that a FIWARE `PATCH` of a
   native bool/number republishes correctly on the ROS side.
3. **`value_mapping` (topic 4, `"ON"→true`):** not expressible in the DDS bridge.
   Resolution options, in order of preference:
   - **(B-i) Native upstream:** the source writes a real boolean into the NGSI-LD
     attribute; `value_mapping` becomes unnecessary. Preferred for a clean DDS story.
   - **(B-ii) Keep buttons on the node:** if the M5Stack firmware can only emit
     `"ON"/"OFF"` strings, leave the IoT-button mappings on the node backend and only
     migrate `angle`. Documented as a deliberate partial migration.
   - Do **not** reintroduce a translation shim in the DDS path — that recreates the
     custom node we are trying to retire.
4. **Risk:** if the Orion-LD DDS module rejects non-string primitives, Class B collapses
   back to node-only; capture that result in this doc and stop the class there.

### Class C — `String` blocked by the reserved `status` leaf (topic 3)
**Strategy: rename the publisher's topic leaf; do not fight the reserved name.**

The Orion-LD DDS module reserves the leaf `status` (collides with ROS 2 action
`status`/`action_msgs/GoalStatusArray`). Confirmed empirically: `…/status2`,
`…/state`, `…/mystatus` bridge; `…/status` never applies.

- Rename the published topic leaf `…/status` → `…/status_json` (the ARISE example's own
  convention) **in the publishing node**. On `shareable-modules` this was out of scope
  (frozen); on this branch it is in scope.
- **Action item for this class:** locate the node that publishes
  `/system_skill_pick_and_place/status`, change the topic name (or add a remap), rebuild,
  and re-run the DDS round-trip on the new leaf. Update `bridge_config.yaml` +
  `generate_config.py` output accordingly.
- Keep a node-side compatibility note: anything still subscribing to the old `…/status`
  leaf must be remapped too (grep before renaming).

### Class D — `custom_interfaces/*` and actions — DESIGN ONLY
Not currently bridged; included so the "all eligible topics" goal is honestly bounded.

- **Actions (`Move.action`)** decompose into ~5 DDS topics (goal/result/cancel/feedback/
  status under `rq/`, `rr/`, `rt/`). They are **not** representable as a single NGSI-LD
  attribute. **Recommendation: out of scope** — keep actions on ROS 2; if a FIWARE-
  visible signal is needed, publish a derived `std_msgs/String` JSON mirror on a
  non-`status` leaf (falls back to Class A).
- **Custom *messages* of primitive fields** (not actions): the DDS bridge's structural
  introspection should map each member to `<attr>.value.<member>` — analogous to the
  commented-out `geometry_msgs/Point → position_x/y/z` example in `bridge_config.yaml`.
  This is the only genuinely new mapping shape and should be prototyped last, with a
  purpose-built primitive msg, only if a real demonstrator need exists.

---

## Proposed implementation order (each gated on hello-world validation)

1. **Class B — `Int32` (`/angle`)** first: simplest non-String, no `value_mapping`.
   Proves the dynamic-type hypothesis with the least confounding.
2. **Class B — `Bool` (`/start_button`, `/stop_button`)**: then resolve `value_mapping`
   per B-i / B-ii.
3. **Class C — `status_json` rename**: touches a production publisher; do it on its own
   so the diff is reviewable in isolation.
4. **Class D — design write-up / optional primitive-msg prototype** last, only if needed.

Each step: extend/run `generate_config.py` → `docker compose -f docker-compose.dds.yml up`
→ publish from a Vulcanexus container → confirm the NGSI-LD attribute → confirm the
reverse `PATCH`. Record the outcome (✅/❌) in this doc before starting the next step.

---

## Validation results (this branch)

All against `fiware/orion-ld:1.13.0-PRE-1835`, publishing from
`eprosima/vulcanexus:jazzy-desktop`, `ROS_DOMAIN_ID=0`, host networking.

### Step 0 — primitive type proof (throwaway topics, before un-filtering the generator)

| Type | Forward (ROS→FIWARE) | Reverse (PATCH→ROS) | `.value.data` shape | ddsDataType |
|---|---|---|---|---|
| `std_msgs/Int32` | ✅ `42` | ✅ `99` | JSON number | `std_msgs::msg::dds_::Int32_` |
| `std_msgs/Bool` | ✅ `true` | ✅ `false` | JSON bool | `std_msgs::msg::dds_::Bool_` |

→ **Hypothesis confirmed: no new bridge code needed.** Generator un-filtered from
`std_msgs/String`-only to all scalar `std_msgs` (`ELIGIBLE_STD_MSGS`).

### Real HARMONY topics — all six now bridge

| Class | ROS 2 topic | Type | Forward | Reverse | Note |
|---|---|---|---|---|---|
| A | `/user_inputs/voice_command` | String | ✅ | ✅ | unchanged |
| A | `/user_inputs/gesture_command` | String | ✅ | ✅ | unchanged |
| B | `/angle` | Int32 | ✅ `137` | ✅ `270` | no ROS consumer in-repo (telemetry only) |
| B | `/user_inputs/start_button` | Bool | ✅ `true` | ✅ `true` | retargeted + native bool |
| B | `/user_inputs/stop_button` | Bool | ✅ | ✅ | structurally identical to start_button |
| C | `/system_skill_pick_and_place/status_json` | String | ✅ `PICKING` | ✅ `DONE` | renamed leaf un-blocks it |

## Decisions taken (resolved during implementation)

1. **Buttons `value_mapping` (Class B) → B-i (native upstream), DROPPED.** Grep showed the
   only ROS consumer is `task_pack_bottle.py` (`start_button_callback`/`stop_button_callback`),
   which depends on a **native boolean** (`if not msg.data: return`) — satisfied by the DDS
   path's native bool. NGSI-v2 dashboards / QuantumLeap read `buttonBlue|buttonRed` from the
   **node-backend Orion stack**, not the DDS Orion-LD, so they are out of the DDS path's scope.
   - **Also fixed a pre-existing mismatch:** the config mapped `/start_button`,`/stop_button`
     but the consumer subscribes to `/user_inputs/start_button`,`/user_inputs/stop_button`
     (no remap exists). Retargeted the mapping to the consumer's real topics so both backends
     deliver. Consumer code unchanged → atomic.
2. **`status` rename (Class C) → done in config only.** Grep found **no in-repo ROS producer**
   of `/system_skill_pick_and_place/status` (in-repo status pubs are
   `/xarm_pack_bottle/robot_status`, `/task_pack_bottle/status`); on the ROS side only the
   bridge subscribes. Renamed the ROS leaf to `status_json` (FIWARE attribute kept as `status`,
   so the dashboard reading attribute `status` is unaffected). The demonstrator's skill node in
   `harmony-dev` must publish on the `status_json` leaf for this to carry real data.
3. **Class D → design-only, confirmed.** Actions are not bridged. No primitive-msg prototype
   built (no current demonstrator need); the field-per-member mapping shape is documented above
   for when one arises.

## Net effect

The DDS path now covers **all six** topics the node backend bridges — it is no longer a
String-only partial alternative. Remaining non-DDS surface = `custom_interfaces/*` actions
(Class D, out of scope by design). The custom node stays available as the `node` backend.
