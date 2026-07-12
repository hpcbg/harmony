#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# validate_dds_command_flow.sh
#
# Validates the DDS-native AI-detector COMMAND FLOW end to end:
#
#     NGSI-LD command=START  →  Orion-LD DDS writer  →  rt/bottle_detection/command
#       →  detector subscription (ros2_backend.py, spin mode)
#       →  /bottle_detection/{status_json,bottle_count,pick_pose_json,result_json}
#       →  Orion-LD attributes (status / bottleCount / pickPose / result)
#
# Unlike validate_dds_native_demo.sh (which uses the TEST_DETECTION_COMMAND
# shortcut that calls run_detection_stub directly), this script drives the REAL
# subscription path: it starts the detector long-running (AI_BACKEND=ros2, no TEST
# env), injects START by updating the Orion-LD `command` attribute, and proves the
# detector reacted by capturing fresh messages on every output topic (the echoes
# start before the trigger, so anything they catch is caused by the command).
#
# It is read-mostly: it only starts the DDS broker (if not already up) and one
# detector node. It does NOT start any NGSI-v2 component and does NOT touch
# launch_pack_bottle.sh.
#
# The command attribute must be written in std_msgs/String DDS form — the value is
# the struct member, i.e. {"value":{"data":"START"}} — a plain string does NOT
# propagate to DDS (validated empirically on this stack).
#
# Recommended runtime: Vulcanexus Jazzy (reliable TypeObject propagation, both
# directions). Run it inside the Vulcanexus image with:
#     ./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
WS="$SCRIPT_DIR/ros2-xarm-pack-bottle/ros2_ws"
DDS_DIR="$WS/src/fiware_bridge/dds"
ORION="${ORION:-http://localhost:1026}"
ENTITY="urn:ngsi-ld:BottleDetectionJob:processor-01"
ECHO_SECS="${ECHO_SECS:-15}"     # how long each output-topic echo listens
READY_WAIT="${READY_WAIT:-15}"   # max seconds to wait for the detector subscription
DISCOVERY_WAIT=3                 # seconds to let the echoes discover before triggering

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

FAILURES=0
DET_PID=""
DET_LOG="$(mktemp)"

cleanup() {
    [ -n "$DET_PID" ] && kill "$DET_PID" >/dev/null 2>&1
    pkill -f "ros2_backend.py" >/dev/null 2>&1
    rm -f "$DET_LOG"
}
trap cleanup EXIT

# ── source a ROS 2 environment (Vulcanexus first, plain Jazzy fallback) ────────
ROS_SETUP=""
for s in /opt/vulcanexus/jazzy/setup.bash /opt/ros/jazzy/setup.bash; do
    if [ -f "$s" ]; then
        # shellcheck disable=SC1090
        source "$s"; ROS_SETUP="$s"; break
    fi
done
if [ -z "$ROS_SETUP" ]; then
    echo "ERROR: no ROS 2 environment found (looked for Vulcanexus Jazzy and ROS 2 Jazzy)."
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
head "DDS-native AI command-flow validation"
info "ROS env       : $ROS_SETUP"
info "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
info "Orion-LD      : $ORION"
info "entity        : $ENTITY"
if [ "$VULCANEXUS" -eq 1 ]; then
    pass "Vulcanexus detected — full DDS type propagation expected (both directions)."
else
    warn "Plain ROS 2 Jazzy (no Vulcanexus): the command may not propagate from Orion-LD"
    info "to DDS, and output values may stay \"uninitialized\". Use Vulcanexus for ARISE."
fi

# ── 1. DDS Orion-LD broker ───────────────────────────────────────────────────
head "1. DDS Orion-LD broker"
ver="$(curl -s --max-time 3 "$ORION/version" 2>/dev/null)"
if printf '%s' "$ver" | grep -qi 'orionld'; then
    pass "DDS Orion-LD broker already running on $ORION"
elif printf '%s' "$ver" | grep -qi 'orion'; then
    fail "Port 1026 is served by a NON-LD Orion (NGSI-v2 stack?). Stop it first."
    exit 1
else
    warn "DDS broker not reachable — starting it (docker compose up -d) …"
    ( cd "$DDS_DIR" && docker compose -f docker-compose.dds.yml up -d ) >/dev/null 2>&1
    for i in $(seq 1 20); do
        curl -s --max-time 2 "$ORION/version" 2>/dev/null | grep -qi 'orionld' \
            && { pass "DDS Orion-LD broker up after ${i}s"; break; }
        sleep 1
    done
    curl -s --max-time 2 "$ORION/version" 2>/dev/null | grep -qi 'orionld' \
        || { fail "DDS broker did not come up on $ORION"; exit 1; }
fi

# ── 2. start the detector (real subscription, spin mode) ─────────────────────
head "2. AI detector (AI_BACKEND=ros2, spin mode)"
info "starting: ( cd ai-bottle-detector-fiware && AI_BACKEND=ros2 ./run.sh )"
( cd "$SCRIPT_DIR/ai-bottle-detector-fiware" && AI_BACKEND=ros2 ./run.sh ) >"$DET_LOG" 2>&1 &
DET_PID=$!

# wait until the detector's subscription to the command topic is discovered
ready=0
for _ in $(seq 1 "$READY_WAIT"); do
    subs="$(ros2 topic info /bottle_detection/command 2>/dev/null \
            | awk -F': ' '/Subscription count/{print $2}')"
    if [ "${subs:-0}" -ge 1 ]; then ready=1; break; fi
    if ! kill -0 "$DET_PID" 2>/dev/null; then break; fi
    sleep 1
done
if [ "$ready" -eq 1 ]; then
    pass "detector subscribed to /bottle_detection/command"
else
    fail "detector did not subscribe to /bottle_detection/command within ${READY_WAIT}s"
    info "detector log:"; sed 's/^/      /' "$DET_LOG"
    exit 1
fi

# ── 3. listen on every output topic, then inject the command from Orion-LD ────
head "3. command flow: Orion-LD command=START → DDS → detector → outputs"

# "ros_topic|echo_msg_type|orion_attr"
specs=(
    "/bottle_detection/status_json|std_msgs/msg/String|status"
    "/bottle_detection/bottle_count|std_msgs/msg/Int32|bottleCount"
    "/bottle_detection/pick_pose_json|std_msgs/msg/String|pickPose"
    "/bottle_detection/result_json|std_msgs/msg/String|result"
)
declare -A OUT PIDS
for spec in "${specs[@]}"; do
    IFS='|' read -r topic mtype attr <<<"$spec"
    f="$(mktemp)"; OUT[$topic]="$f"
    ( timeout "$ECHO_SECS" ros2 topic echo "$topic" "$mtype" > "$f" 2>&1 ) &
    PIDS[$topic]=$!
done
sleep "$DISCOVERY_WAIT"

# trigger: write the command attribute in std_msgs/String DDS form (member `data`)
info "PATCH $ENTITY.command = {\"data\":\"START\"}"
code="$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 \
    -X PATCH "$ORION/ngsi-ld/v1/entities/$ENTITY/attrs/command" \
    -H 'Content-Type: application/json' \
    -d '{"type":"Property","value":{"data":"START"}}')"
if [ "$code" = "204" ] || [ "$code" = "207" ]; then
    pass "Orion-LD accepted the command update (HTTP $code)"
else
    fail "Orion-LD rejected the command update (HTTP $code)"
    FAILURES=$((FAILURES + 1))
fi

for spec in "${specs[@]}"; do
    IFS='|' read -r topic mtype attr <<<"$spec"
    wait "${PIDS[$topic]}" 2>/dev/null
done

# ── 4. check each output topic reacted, and the mapped Orion-LD value ─────────
orion_classify() {
    local attr="$1" body
    body="$(curl -s --max-time 5 \
        "$ORION/ngsi-ld/v1/entities/$ENTITY?local=true&attrs=$attr" \
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

head "4. results"
declare -A R_TOPIC R_ORION
for spec in "${specs[@]}"; do
    IFS='|' read -r topic mtype attr <<<"$spec"
    f="${OUT[$topic]}"
    if grep -q '^data:' "$f"; then
        seen="$(grep '^data:' "$f" | tail -1 | sed 's/^data: //')"
        pass "topic $topic reacted: $seen"
        R_TOPIC[$attr]="PASS"
    else
        fail "topic $topic: no message after the command within ${ECHO_SECS}s"
        R_TOPIC[$attr]="FAIL"
        FAILURES=$((FAILURES + 1))
    fi
    rm -f "$f"

    cls="$(orion_classify "$attr")"
    case "$cls" in
        REAL*)         pass "Orion-LD $attr = REAL ${cls#REAL }"; R_ORION[$attr]="REAL ${cls#REAL }" ;;
        UNINITIALIZED) warn "Orion-LD $attr = \"uninitialized\""; R_ORION[$attr]="UNINITIALIZED" ;;
        MISSING)       warn "Orion-LD $attr not found";           R_ORION[$attr]="MISSING" ;;
        *)             warn "Orion-LD $attr query failed";        R_ORION[$attr]="ERROR" ;;
    esac
done

# ── 5. summary ───────────────────────────────────────────────────────────────
head "Summary — command flow (Orion-LD → DDS → detector → outputs)"
printf '  %-14s %-8s %s\n' "OUTPUT" "TOPIC" "ORION-LD VALUE"
hr
for attr in status bottleCount pickPose result; do
    printf '  %-14s %-8s %s\n' ".$attr" "${R_TOPIC[$attr]:-?}" "${R_ORION[$attr]:-?}"
done
hr
if [ "$FAILURES" -eq 0 ]; then
    pass "Command flow works: an Orion-LD command=START drove the detector over DDS."
else
    fail "$FAILURES check(s) FAILED."
    [ "$VULCANEXUS" -eq 0 ] && info "→ try Vulcanexus: ./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh"
fi

# ── optional: control-loop latency measurement (MEASURE_DDS_LATENCY=1) ────────
# The command-flow test above is unchanged. This measures the pure ROS → FIWARE →
# ROS communication loop and intentionally excludes AI detection time.
if [ "${MEASURE_DDS_LATENCY:-0}" = "1" ] && [ "$FAILURES" -eq 0 ]; then
    head "DDS/FIWARE control-loop latency (MEASURE_DDS_LATENCY=1)"
    info "running ./measure_dds_fiware_latency.sh (excludes AI detection time) …"
    "$SCRIPT_DIR/measure_dds_fiware_latency.sh" || warn "latency measurement reported an issue (non-fatal)"
fi

exit "$FAILURES"
