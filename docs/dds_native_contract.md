# DDS-native Orion-LD contract (dashboard / backend team)

This document is the **validated** NGSI-LD contract exposed by the DDS-native HRI
modules: the values the dashboard/backend can read from, and the command it can
write to, the Orion-LD context broker. Every shape below was confirmed end-to-end
against a running broker (see [Validation](#validation)).

> **Read this first — value nesting.** The Orion-LD DDS bridge wraps every ROS 2
> `std_msgs` payload under `value.data`. This is true for **both** `String` and
> `Int32` topics; only the *type of the leaf* differs:
> - `std_msgs/String`  → `value.data` is a **JSON string**
> - `std_msgs/Int32`   → `value.data` is a **JSON number**
>
> So read `…value.data` in all cases (including `bottleCount`). Alongside `value`,
> each attribute also carries DDS metadata siblings (`ddsDataType`, `participantId`,
> `instanceHandleId`, `publishedAt`) that can be ignored by consumers.

---

## Runtime

| Item | Value |
|---|---|
| Branch | `dds-full-integration` |
| Context broker | **Orion-LD** started with `-wip dds -mongocOnly` (NGSI-LD only; returns HTTP 501 for NGSI-v2) |
| Broker endpoint | `http://localhost:1026` |
| ROS side | **Vulcanexus Jazzy** (the Docker image `eprosima/vulcanexus:jazzy-desktop`), `ROS_DOMAIN_ID=0` |
| Why Vulcanexus | The FIWARE DDS Enabler only fills an NGSI-LD value once the publisher propagates the `std_msgs::msg::dds_::*` TypeObject. Plain ROS 2 Jazzy publishes the topic but does not reliably propagate the type, so values stay `"uninitialized"`. Vulcanexus propagates it reliably (both directions). |

A value of `"uninitialized"` (a plain string, no `value.data`) means the attribute
is mapped but no DDS sample has been received/propagated yet — treat it as "no data".

---

## Validation

The contract is reproducible with the repo-root scripts (broker is started on the
host, validation runs inside the Vulcanexus image with host networking + IPC):

```bash
# Voice + Gesture + AI-detector outputs (self-tests → topics → Orion-LD values)
./run_vulcanexus_dds_validation.sh

# AI-detector COMMAND flow: NGSI-LD command=START → DDS → detector → outputs → Orion-LD
./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh
```

Both report each Orion-LD value as **REAL** or **"uninitialized"**.

---

## Entities and attributes

All reads use the NGSI-LD entities endpoint with `?local=true` and
`Accept: application/json`. No `Fiware-Service`/`Fiware-ServicePath` headers are
required on the DDS Orion-LD broker.

### 1. `urn:ngsi-ld:VoiceCommand:operator-1`

| Attribute | Path to value | Type | Allowed values |
|---|---|---|---|
| `command` | `command.value.data` | string | `PICK` · `GO` · `STOP` · `CAP` · `GIVE` · `SAFE` · `FAST` |

```json
"command": { "type": "Property", "value": { "data": "PICK" } }
```

### 2. `urn:ngsi-ld:GestureDetector:operator-1`

| Attribute | Path to value | Type | Allowed values |
|---|---|---|---|
| `command` | `command.value.data` | string | `NO_HAND` · `CAP_PLACED` · `SIDE_GRIP` |

```json
"command": { "type": "Property", "value": { "data": "SIDE_GRIP" } }
```

### 3. `urn:ngsi-ld:BottleDetectionJob:processor-01`

| Attribute | Path to value | Type | Notes |
|---|---|---|---|
| `command` | `command.value.data` | string | **Input** (written by the dashboard). Accepts `START`. See [Sending a command](#sending-a-command-ngsi-ld-patch). |
| `status` | `status.value.data` | string (JSON) | e.g. `{"status": "DONE", "bottleCount": 1}`; lifecycle: `CAPTURING` → `PROCESSING` → `DONE` (or `FAILED`). |
| `bottleCount` | `bottleCount.value.data` | **number** | Detected bottle count, e.g. `1`. (`std_msgs/Int32` → numeric leaf.) |
| `pickPose` | `pickPose.value.data` | string (JSON) | e.g. `{"x": 120.0, "y": -45.0, "rotation": 30.0}`; `{}` when no pickable object. |
| `result` | `result.value.data` | string (JSON) | Full result, e.g. `{"status":"DONE","bottleCount":1,"pickPose":{…},"bottles":[…]}`. |

`status`, `pickPose` and `result` carry **JSON encoded as a string** in `value.data`
— the consumer must `JSON.parse(value.data)` to get the structured object.

Example attribute shapes as returned by Orion-LD:

```json
"status":      { "type": "Property", "value": { "data": "{\"status\": \"DONE\", \"bottleCount\": 1}" } },
"bottleCount": { "type": "Property", "value": { "data": 1 } },
"pickPose":    { "type": "Property", "value": { "data": "{\"x\": 120.0, \"y\": -45.0, \"rotation\": 30.0}" } },
"result":      { "type": "Property", "value": { "data": "{\"status\": \"DONE\", \"bottleCount\": 1, \"pickPose\": {\"x\": 120.0, \"y\": -45.0, \"rotation\": 30.0}, \"bottles\": [{\"x\": 120.0, \"y\": -45.0, \"yaw\": 30.0, \"conf\": 0.99, \"selected\": true}]}" } }
```

> Not on the DDS path (still NGSI-v2 / node backend only): processed-image URLs and
> base64 payloads, and the decomposed `pickX`/`pickY`/`pickRotation` floats. On DDS
> the pose travels as one JSON string in `pickPose`.

The ROS-side consumer of these outputs is the `task_pack_bottle` behaviour tree: with
`TASK_DETECTOR_SOURCE=dds` it subscribes to the ROS 2 topic `/bottle_detection/result_json`
(the atomic full result) and drives its pick stage from `status == "DONE"` + `pickPose`.
The dashboard/backend reads the same data as Orion-LD attributes (table above).

---

## Reading entities (curl)

```bash
# 1. Voice command
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:VoiceCommand:operator-1?local=true" \
  -H 'Accept: application/json'

# 2. Gesture command
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:GestureDetector:operator-1?local=true" \
  -H 'Accept: application/json'

# 3. Bottle detection job (all attributes)
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:BottleDetectionJob:processor-01?local=true" \
  -H 'Accept: application/json'
```

Fetch only specific attributes with `&attrs=`:

```bash
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:BottleDetectionJob:processor-01?local=true&attrs=status,bottleCount,pickPose,result" \
  -H 'Accept: application/json'
```

Extracting the actual values (note `value.data` in every case):

```bash
E=urn:ngsi-ld:BottleDetectionJob:processor-01
BASE="http://localhost:1026/ngsi-ld/v1/entities/$E?local=true"

# string-JSON attribute → parse twice (outer NGSI-LD, inner payload string)
curl -s "$BASE&attrs=status" -H 'Accept: application/json' | jq -r '.status.value.data | fromjson'

# numeric attribute → read directly
curl -s "$BASE&attrs=bottleCount" -H 'Accept: application/json' | jq '.bottleCount.value.data'

# voice / gesture keyword
curl -s "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:VoiceCommand:operator-1?local=true" \
  -H 'Accept: application/json' | jq -r '.command.value.data'
```

---

## Sending a command (NGSI-LD PATCH)

The AI detector's `command` is an **outbound** attribute: Orion-LD is the DDS writer,
so updating it publishes a `std_msgs/String` on `rt/bottle_detection/command`, which
the running detector receives and acts on (`START` → run detection → emit
status/count/pose/result).

**The value must be written in `std_msgs/String` DDS form — i.e. `value.data`.**
A plain string value (`"value": "START"`) does **not** propagate to DDS.

```bash
curl -s -X PATCH \
  "http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:BottleDetectionJob:processor-01/attrs/command" \
  -H 'Content-Type: application/json' \
  -d '{"type":"Property","value":{"data":"START"}}'
# → HTTP 204
```

Body shape:

```json
{ "type": "Property", "value": { "data": "START" } }
```

The detector must be running (`AI_BACKEND=ros2 ./run.sh`, stub or
`AI_DETECTION_MODE=real`) for the command to have an effect; the outputs above then
update on the same entity.
