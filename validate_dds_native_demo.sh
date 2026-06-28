#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# validate_dds_native_demo.sh
#
# End-to-end check of the DDS-native (AI_BACKEND/VOICE_BACKEND/GESTURE_BACKEND=ros2)
# components against the Orion-LD DDS broker. It is read-mostly: it only starts the
# DDS broker (if not already up) and runs the three no-mic/no-camera self-tests.
#
# It DOES NOT start any NGSI-v2 component (the FastAPI detector service, the React
# dashboard, the standard Orion stack, QuantumLeap/Crate/Grafana) and DOES NOT
# touch launch_pack_bottle.sh.
#
# For each migrated component it:
#   1. runs the ros2 backend self-test (TEST_* env, no audio/camera/GPU/torch),
#   2. confirms the std_msgs/String reached the ROS 2 topic (ros2 topic echo),
#   3. queries the mapped Orion-LD NGSI-LD entity and reports whether the value is
#      REAL or still "uninitialized".
#
# NOTE on "uninitialized": the FIWARE DDS Enabler only fills a value once the
# publisher propagates the std_msgs::msg::dds_::String_ TypeObject. That needs
# Vulcanexus Jazzy on the ROS side; plain ROS 2 Jazzy publishes the topic (so the
# ros2-topic check passes) but does not propagate the type (so the Orion value
# stays "uninitialized"). This script detects and explains that case.
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
WS="$SCRIPT_DIR/ros2-xarm-pack-bottle/ros2_ws"
DDS_DIR="$WS/src/fiware_bridge/dds"
ORION="${ORION:-http://localhost:1026}"
ECHO_SECS="${ECHO_SECS:-12}"     # how long each ros2 topic echo listens
DISCOVERY_WAIT=3                 # seconds to let echo discover before publishing

# ── pretty output ────────────────────────────────────────────────────────────
if [ -t 1 ]; then
    BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'
    CYN=$'\033[36m'; RST=$'\033[0m'
else
    BOLD=''; RED=''; GRN=''; YEL=''; CYN=''; RST=''
fi
hr()   { printf '%s\n' "────────────────────────────────────────────────────────────"; }
head() { printf '\n%s%s%s\n' "$BOLD$CYN" "$1" "$RST"; hr; }
pass() { printf '  %s✓%s %s\n' "$GRN" "$RST" "$1"; }
warn() { printf '  %s!%s %s\n' "$YEL" "$RST" "$1"; }
fail() { printf '  %s✗%s %s\n' "$RED" "$RST" "$1"; }
info() { printf '    %s\n' "$1"; }

TOPIC_FAILURES=0
declare -A RESULT_TOPIC
declare -A RESULT_ORION

# ── source a ROS 2 environment (Vulcanexus first, plain Jazzy fallback) ───────
ROS_SETUP=""
for s in /opt/vulcanexus/jazzy/setup.bash /opt/ros/jazzy/setup.bash; do
    if [ -f "$s" ]; then
        # shellcheck disable=SC1090
        source "$s"; ROS_SETUP="$s"; break
    fi
done
if [ -z "$ROS_SETUP" ]; then
    echo "ERROR: no ROS 2 environment found (looked for Vulcanexus Jazzy and ROS 2 Jazzy)."
    echo "       Install one before running the DDS-native validation."
    exit 1
fi
# shellcheck disable=SC1090
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "$ROS_SETUP" in
    */vulcanexus/*) VULCANEXUS=1 ;;
    *)              VULCANEXUS=0 ;;
esac

PY="$(command -v python3 || command -v python)"

# ── header ───────────────────────────────────────────────────────────────────
head "DDS-native demo validation"
info "ROS env       : $ROS_SETUP"
info "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
info "Orion-LD      : $ORION"
if [ "$VULCANEXUS" -eq 1 ]; then
    pass "Vulcanexus detected — full DDS type propagation expected."
else
    warn "Plain ROS 2 Jazzy (no Vulcanexus): topics will publish, but Orion-LD values"
    info "may stay \"uninitialized\" because the std_msgs::String TypeObject is not propagated."
fi

# ── 1. DDS Orion-LD broker ───────────────────────────────────────────────────
head "1. DDS Orion-LD broker"
ver="$(curl -s --max-time 3 "$ORION/version" 2>/dev/null)"
if printf '%s' "$ver" | grep -qi 'orionld'; then
    pass "DDS Orion-LD broker already running on $ORION"
elif printf '%s' "$ver" | grep -qi 'orion'; then
    fail "Port 1026 is served by a NON-LD Orion (NGSI-v2 stack?). Stop it first —"
    info "this validation must run against the DDS Orion-LD broker only."
    exit 1
else
    warn "DDS broker not reachable — starting it (docker compose up -d) …"
    ( cd "$DDS_DIR" && docker compose -f docker-compose.dds.yml up -d ) >/dev/null 2>&1
    for i in $(seq 1 20); do
        if curl -s --max-time 2 "$ORION/version" 2>/dev/null | grep -qi 'orionld'; then
            pass "DDS Orion-LD broker up after ${i}s"; break
        fi
        sleep 1
    done
    if ! curl -s --max-time 2 "$ORION/version" 2>/dev/null | grep -qi 'orionld'; then
        fail "DDS broker did not come up on $ORION"; exit 1
    fi
fi

# ── helper: classify an Orion-LD attribute value ─────────────────────────────
# echoes one of:  REAL <json>   |   UNINITIALIZED   |   MISSING   |   ERROR
orion_classify() {
    local entity="$1" attr="$2" body
    body="$(curl -s --max-time 5 \
        "$ORION/ngsi-ld/v1/entities/$entity?local=true&attrs=$attr" \
        -H 'Accept: application/json' 2>/dev/null)"
    printf '%s' "$body" | "$PY" -c '
import sys, json
attr = sys.argv[1]
try:
    d = json.load(sys.stdin)
except Exception:
    print("ERROR"); sys.exit()
if not isinstance(d, dict) or "id" not in d:
    print("MISSING"); sys.exit()
a = d.get(attr)
if a is None:
    print("MISSING"); sys.exit()
v = a.get("value") if isinstance(a, dict) else a
if v == "uninitialized":
    print("UNINITIALIZED")
elif isinstance(v, dict) and "data" in v:
    print("REAL " + json.dumps(v["data"]))
else:
    print("REAL " + json.dumps(v))
' "$attr"
}

# ── helper: run one component test + topic echo + orion check ────────────────
# args: LABEL  DIR  TOPIC  ENTITY  ATTR  ENV1 ENV2 ...
validate_component() {
    local label="$1" dir="$2" topic="$3" entity="$4" attr="$5"; shift 5
    local echo_out; echo_out="$(mktemp)"

    head "$label"
    info "self-test : ( cd $(basename "$dir") && $* ./run.sh )"
    info "ros topic : $topic"
    info "entity    : $entity ($attr)"

    # start the topic listener, then run the component self-test
    ( timeout "$ECHO_SECS" ros2 topic echo "$topic" std_msgs/msg/String \
        > "$echo_out" 2>&1 ) &
    local echo_pid=$!
    sleep "$DISCOVERY_WAIT"

    ( cd "$dir" && env "$@" ./run.sh ) 2>&1 | sed 's/^/      /'
    wait "$echo_pid" 2>/dev/null

    # topic check
    if grep -q '^data:' "$echo_out"; then
        local seen; seen="$(grep '^data:' "$echo_out" | tail -1 | sed 's/^data: //')"
        pass "ROS 2 topic $topic delivered: $seen"
        RESULT_TOPIC[$label]="PASS"
    else
        fail "ROS 2 topic $topic: no message received within ${ECHO_SECS}s"
        RESULT_TOPIC[$label]="FAIL"
        TOPIC_FAILURES=$((TOPIC_FAILURES + 1))
    fi
    rm -f "$echo_out"

    # orion check
    local cls; cls="$(orion_classify "$entity" "$attr")"
    case "$cls" in
        REAL*)
            pass "Orion-LD $attr = REAL ${cls#REAL }"
            RESULT_ORION[$label]="REAL ${cls#REAL }" ;;
        UNINITIALIZED)
            warn "Orion-LD $attr = \"uninitialized\" (entity+attr mapped, value not propagated)"
            if [ "$VULCANEXUS" -eq 0 ]; then
                info "→ expected without Vulcanexus: run under Vulcanexus Jazzy to initialise it."
            fi
            RESULT_ORION[$label]="UNINITIALIZED" ;;
        MISSING)
            warn "Orion-LD entity/attribute not found (is the rt/ mapping in context_broker_config.json?)"
            RESULT_ORION[$label]="MISSING" ;;
        *)
            warn "Orion-LD query failed or returned unexpected payload"
            RESULT_ORION[$label]="ERROR" ;;
    esac
}

# ── 2-4. the three migrated DDS-native components ────────────────────────────
validate_component "2. Voice (VOICE_BACKEND=ros2)" \
    "$SCRIPT_DIR/voice-commands-fiware" \
    "/user_inputs/voice_command" \
    "urn:ngsi-ld:VoiceCommand:operator-1" "command" \
    VOICE_BACKEND=ros2 TEST_KEYWORD=PICK

validate_component "3. Gesture (GESTURE_BACKEND=ros2)" \
    "$SCRIPT_DIR/gesture-commands-fiware" \
    "/user_inputs/gesture_command" \
    "urn:ngsi-ld:GestureDetector:operator-1" "command" \
    GESTURE_BACKEND=ros2 TEST_GESTURE=SIDE_GRIP

validate_component "4. AI detector (AI_BACKEND=ros2)" \
    "$SCRIPT_DIR/ai-bottle-detector-fiware" \
    "/bottle_detection/status_json" \
    "urn:ngsi-ld:BottleDetectionJob:processor-01" "status" \
    AI_BACKEND=ros2 TEST_DETECTION_COMMAND=START

# ── 5. summary ───────────────────────────────────────────────────────────────
head "Summary"
printf '  %-38s %-8s %s\n' "COMPONENT" "ROS2" "ORION-LD VALUE"
hr
for label in \
    "2. Voice (VOICE_BACKEND=ros2)" \
    "3. Gesture (GESTURE_BACKEND=ros2)" \
    "4. AI detector (AI_BACKEND=ros2)"; do
    t="${RESULT_TOPIC[$label]:-?}"
    o="${RESULT_ORION[$label]:-?}"
    printf '  %-38s %-8s %s\n' "$label" "$t" "$o"
done
hr

if [ "$TOPIC_FAILURES" -eq 0 ]; then
    pass "All ROS 2 topic checks passed (DDS-native publish path works)."
else
    fail "$TOPIC_FAILURES ROS 2 topic check(s) FAILED."
fi
if [ "$VULCANEXUS" -eq 0 ]; then
    warn "Orion-LD values may read \"uninitialized\" on plain ROS 2 Jazzy — that is the"
    info "documented Vulcanexus type-propagation requirement, not a component failure."
fi

exit "$TOPIC_FAILURES"
