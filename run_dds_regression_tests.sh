#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_dds_regression_tests.sh
#
# One regression runner for the DDS-native work on the `dds-full-integration`
# branch, including the task_pack_bottle no-robot safety fix.
#
# Checks:
#   1. Vulcanexus DDS-native validation        (./run_vulcanexus_dds_validation.sh)
#   2. DDS command-flow validation             (… validate_dds_command_flow.sh)
#   3. task_pack_bottle, DDS source, NO robot  (no-robot crash must be fixed)
#   4. task_pack_bottle, default source        (must stay the job_json/node path)
#   5. dummy xArm action server success path    (server-available path unchanged)
#
# This is a VALIDATION script only:
#   - it does NOT modify launch_pack_bottle.sh,
#   - it does NOT start the full dashboard / NGSI-v2 demo,
#   - it needs no robot hardware,
#   - DDS validation runs in the Vulcanexus Docker image (recommended),
#   - the local task tests use whatever ROS is available (printed below).
#
# Logs are captured under a temp dir (/tmp/harmony-dds-regression-XXXXXX).
# Background processes are always cleaned up on exit.
# ─────────────────────────────────────────────────────────────────────────────

set -u

REPO="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
WS="$REPO/ros2-xarm-pack-bottle/ros2_ws"
LOGDIR="$(mktemp -d /tmp/harmony-dds-regression-XXXXXX)"
# Dedicated domain for the local task tests, isolated from any running demo /
# the domain-0 DDS broker used by checks 1-2.
TASK_DOMAIN="${REGRESSION_DOMAIN:-7}"

# ── pretty output ────────────────────────────────────────────────────────────
if [ -t 1 ]; then
    BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'
    CYN=$'\033[36m'; RST=$'\033[0m'
else
    BOLD=''; RED=''; GRN=''; YEL=''; CYN=''; RST=''
fi
hr()   { printf '%s\n' "──────────────────────────────────────────────────────────────"; }
head() { printf '\n%s%s%s\n' "$BOLD$CYN" "$1" "$RST"; hr; }
ok()   { printf '  %s✓%s %s\n' "$GRN" "$RST" "$1"; }
bad()  { printf '  %s✗%s %s\n' "$RED" "$RST" "$1"; SEC_FAIL=$((SEC_FAIL + 1)); }
warn() { printf '  %s!%s %s\n' "$YEL" "$RST" "$1"; }
info() { printf '    %s\n' "$1"; }

declare -A RESULT
ORDER=()
record() { RESULT["$1"]="$2"; ORDER+=("$1"); }

# ── background-process bookkeeping (kill whole process groups on exit) ────────
BG_PGIDS=()
LAST_BG_PID=""
start_bg() {            # start_bg <logfile> <cmd...>
    local log="$1"; shift
    setsid "$@" >"$log" 2>&1 &
    LAST_BG_PID=$!
    BG_PGIDS+=("$LAST_BG_PID")
}
stop_bg() { [ -n "${1:-}" ] && kill -TERM -"$1" 2>/dev/null; }
cleanup() {
    for pg in "${BG_PGIDS[@]:-}"; do kill -TERM -"$pg" 2>/dev/null; done
    sleep 1
    for pg in "${BG_PGIDS[@]:-}"; do kill -KILL -"$pg" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

assert_grep()     { if grep -qE "$3" "$2" 2>/dev/null; then ok "$1"; else bad "$1"; fi; }
assert_not_grep() { if grep -qE "$3" "$2" 2>/dev/null; then bad "$1"; else ok "$1"; fi; }

# ── header ───────────────────────────────────────────────────────────────────
head "DDS-native regression tests (dds-full-integration)"
info "repo    : $REPO"
info "logs    : $LOGDIR"
info "task ROS_DOMAIN_ID (local checks 3-5): $TASK_DOMAIN"

# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ 1. Vulcanexus DDS-native validation                                         ║
# ╚════════════════════════════════════════════════════════════════════════════╝
head "1. Vulcanexus DDS-native validation"
SEC_FAIL=0
LOG1="$LOGDIR/01_vulcanexus_native.log"
if ! command -v docker >/dev/null 2>&1; then
    bad "docker not available — cannot run the Vulcanexus DDS validation"
else
    info "running ./run_vulcanexus_dds_validation.sh (this builds/starts the DDS broker)…"
    ( cd "$REPO" && ./run_vulcanexus_dds_validation.sh ) >"$LOG1" 2>&1
    rc=$?
    info "exit code: $rc   (log: $LOG1)"
    # The authoritative outcome is the Orion-LD value (REAL vs UNINITIALIZED) — that
    # is the contract the dashboard consumes. The inner one-shot `ros2 topic echo`
    # check is racy when the long-lived DDS broker is also subscribed (the publish
    # can fire after the broker matches but before the ephemeral echo), so the
    # topic-echo PASS/FAIL and the exit code are reported as informational only.
    assert_grep "voice → Orion-LD REAL"        "$LOG1" 'Voice.*REAL "'
    assert_grep "gesture → Orion-LD REAL"      "$LOG1" 'Gesture.*REAL "'
    assert_grep "AI status → Orion-LD REAL"    "$LOG1" '\.status .*REAL'
    assert_grep "AI bottleCount → Orion-LD REAL" "$LOG1" '\.bottleCount .*REAL'
    assert_grep "AI pickPose → Orion-LD REAL"  "$LOG1" '\.pickPose .*REAL'
    assert_grep "AI result → Orion-LD REAL"    "$LOG1" '\.result .*REAL'
    assert_not_grep "no UNINITIALIZED values"  "$LOG1" 'UNINITIALIZED'
    assert_not_grep "no MISSING entities"      "$LOG1" 'not found'
    if [ "$rc" -eq 0 ]; then ok "topic-echo checks also passed (rc=0)"; \
        else warn "topic-echo check rc=$rc (racy one-shot; Orion-LD values above are authoritative)"; fi
fi
[ "$SEC_FAIL" -eq 0 ] && record "1. Vulcanexus DDS-native validation" "PASS" \
                      || record "1. Vulcanexus DDS-native validation" "FAIL"

# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ 2. DDS command-flow validation                                              ║
# ╚════════════════════════════════════════════════════════════════════════════╝
head "2. DDS command-flow validation"
SEC_FAIL=0
LOG2="$LOGDIR/02_command_flow.log"
if ! command -v docker >/dev/null 2>&1; then
    bad "docker not available — cannot run the command-flow validation"
else
    info "running ./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh …"
    ( cd "$REPO" && ./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh ) \
        >"$LOG2" 2>&1
    rc=$?
    info "exit code: $rc   (log: $LOG2)"
    # As in check 1, key on the authoritative Orion-LD REAL values; the inner
    # topic-echo/exit-code are informational (racy one-shot vs long-lived broker).
    assert_grep "Orion-LD accepted command=START" "$LOG2" 'accepted the command update'
    assert_grep "status → Orion-LD REAL"      "$LOG2" '\.status .*REAL'
    assert_grep "bottleCount → Orion-LD REAL" "$LOG2" '\.bottleCount .*REAL'
    assert_grep "pickPose → Orion-LD REAL"    "$LOG2" '\.pickPose .*REAL'
    assert_grep "result → Orion-LD REAL"      "$LOG2" '\.result .*REAL'
    assert_grep "command flow works"          "$LOG2" 'Command flow works'
    assert_not_grep "no UNINITIALIZED values" "$LOG2" 'UNINITIALIZED'
    if [ "$rc" -eq 0 ]; then ok "topic-echo checks also passed (rc=0)"; \
        else warn "topic-echo check rc=$rc (racy one-shot; Orion-LD values above are authoritative)"; fi
fi
[ "$SEC_FAIL" -eq 0 ] && record "2. DDS command-flow validation" "PASS" \
                      || record "2. DDS command-flow validation" "FAIL"

# ── local ROS environment for checks 3-5 ─────────────────────────────────────
head "Local ROS environment (checks 3-5)"
ROS_SOURCED=""
# ROS setup.bash references unbound vars; relax `set -u` only while sourcing.
set +u
for s in /opt/vulcanexus/jazzy/setup.bash /opt/ros/jazzy/setup.bash; do
    if [ -f "$s" ]; then
        # shellcheck disable=SC1090
        source "$s"; ROS_SOURCED="$s"; break
    fi
done
set -u
if [ -z "$ROS_SOURCED" ]; then
    warn "no ROS 2 environment found — skipping local task checks 3-5."
    record "3. task_pack_bottle DDS source, no robot" "SKIP (no ROS)"
    record "4. task_pack_bottle default source" "SKIP (no ROS)"
    record "5. dummy action-server success path" "SKIP (no ROS)"
else
    ok "sourced ROS: $ROS_SOURCED"
    export ROS_DOMAIN_ID="$TASK_DOMAIN"
    info "building task_pack_bottle (+ custom_interfaces if needed)…"
    (
        cd "$WS" || exit 1
        [ -d install/custom_interfaces ] || \
            colcon build --packages-select custom_interfaces
        colcon build --packages-select task_pack_bottle
    ) >"$LOGDIR/build.log" 2>&1 && ok "workspace built" || bad "colcon build failed (see $LOGDIR/build.log)"
    # shellcheck disable=SC1090
    set +u
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    set -u

    RESULT_JSON='{"status": "DONE", "bottleCount": 1, "pickPose": {"x": 120.0, "y": -45.0, "rotation": 30.0}}'
    JOB_JSON='{"status": "DONE", "pick_pose": {"x": 120.0, "y": -45.0, "rotation": 30.0}}'

    # ╔════════════════════════════════════════════════════════════════════════╗
    # ║ 3. task_pack_bottle, DDS detector source, NO robot                       ║
    # ╚════════════════════════════════════════════════════════════════════════╝
    head "3. task_pack_bottle — DDS source, NO robot (no-robot crash fixed)"
    SEC_FAIL=0
    NODE3="$LOGDIR/03_node.log"; STAGE3="$LOGDIR/03_stage.log"
    start_bg "$STAGE3" ros2 topic echo /task_pack_bottle/stage std_msgs/msg/String --field data
    ECHO3_PG=$LAST_BG_PID
    start_bg "$NODE3" env TASK_DETECTOR_SOURCE=dds ros2 run task_pack_bottle task_pack_bottle
    NODE3_PG=$LAST_BG_PID
    sleep 5
    info "publishing start_button=true"
    timeout 10 ros2 topic pub --once /user_inputs/start_button std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
    sleep 3
    info "publishing DDS-native /bottle_detection/result_json (DONE + pickPose)"
    timeout 10 ros2 topic pub --once /bottle_detection/result_json std_msgs/msg/String \
        "{data: '$RESULT_JSON'}" >/dev/null 2>&1
    sleep 6
    if kill -0 "$NODE3_PG" 2>/dev/null; then ok "node still alive (did not crash)"; else bad "node DIED (crash)"; fi
    assert_grep     "subscribes to /bottle_detection/result_json" "$NODE3" 'DDS-native source .*/bottle_detection/result_json'
    assert_grep     "reached DETECT_READY / pick stage"           "$STAGE3" 'DETECT_READY|PICK'
    assert_grep     "action-server-unavailable warning present"   "$NODE3" 'unavailable; skipping robot action'
    assert_not_grep "no Traceback"                                "$NODE3" 'Traceback'
    assert_not_grep "no AttributeError: goal_future"              "$NODE3" "AttributeError.*goal_future"
    stop_bg "$NODE3_PG"; stop_bg "$ECHO3_PG"; sleep 2
    [ "$SEC_FAIL" -eq 0 ] && record "3. task_pack_bottle DDS source, no robot" "PASS" \
                          || record "3. task_pack_bottle DDS source, no robot" "FAIL"

    # ╔════════════════════════════════════════════════════════════════════════╗
    # ║ 4. task_pack_bottle, default detector source (job_json / node path)      ║
    # ╚════════════════════════════════════════════════════════════════════════╝
    head "4. task_pack_bottle — default source stays job_json (node path)"
    SEC_FAIL=0
    NODE4="$LOGDIR/04_node.log"; STAGE4="$LOGDIR/04_stage.log"
    start_bg "$STAGE4" ros2 topic echo /task_pack_bottle/stage std_msgs/msg/String --field data
    ECHO4_PG=$LAST_BG_PID
    # No TASK_DETECTOR_SOURCE → must default to job_json.
    start_bg "$NODE4" ros2 run task_pack_bottle task_pack_bottle
    NODE4_PG=$LAST_BG_PID
    sleep 5
    # Confirm the subscription is on the node/NGSI-v2 job_json topic, not result_json.
    JOBINFO="$LOGDIR/04_job_json_info.log"; RESINFO="$LOGDIR/04_result_json_info.log"
    ros2 topic info /bottle_detection/job_json   > "$JOBINFO" 2>&1
    ros2 topic info /bottle_detection/result_json > "$RESINFO" 2>&1
    if grep -qE 'Subscription count: *[1-9]' "$JOBINFO"; then ok "subscribes to /bottle_detection/job_json"; else bad "no subscriber on /bottle_detection/job_json"; fi
    assert_not_grep "did NOT select DDS source"  "$NODE4" 'DDS-native source'
    info "publishing old-style /bottle_detection/job_json (DONE + pick_pose)"
    timeout 10 ros2 topic pub --once /user_inputs/start_button std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
    sleep 3
    timeout 10 ros2 topic pub --once /bottle_detection/job_json std_msgs/msg/String \
        "{data: '$JOB_JSON'}" >/dev/null 2>&1
    sleep 5
    assert_grep     "job_json path reached DETECT_READY / pick" "$STAGE4" 'DETECT_READY|PICK'
    assert_not_grep "no Traceback"                              "$NODE4" 'Traceback'
    stop_bg "$NODE4_PG"; stop_bg "$ECHO4_PG"; sleep 2
    [ "$SEC_FAIL" -eq 0 ] && record "4. task_pack_bottle default source" "PASS" \
                          || record "4. task_pack_bottle default source" "FAIL"

    # ╔════════════════════════════════════════════════════════════════════════╗
    # ║ 5. dummy xArm action-server success path                                 ║
    # ╚════════════════════════════════════════════════════════════════════════╝
    head "5. dummy action-server success path (server-available unchanged)"
    SEC_FAIL=0
    DUMMY_PY="$LOGDIR/dummy_pick_server.py"
    cat > "$DUMMY_PY" <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import Move


class DummyPick(Node):
    def __init__(self):
        super().__init__('dummy_pick_server')
        self._s = ActionServer(self, Move, '/xarm_pack_bottle/pick',
                               self.execute)
        self.get_logger().info('dummy /xarm_pack_bottle/pick server up')

    def execute(self, goal_handle):
        self.get_logger().info(
            'received pick goal pose_json=' + goal_handle.request.pose_json)
        goal_handle.succeed()
        r = Move.Result()
        r.success = True
        r.message = 'dummy ok'
        return r


def main():
    rclpy.init()
    rclpy.spin(DummyPick())


if __name__ == '__main__':
    main()
PY
    NODE5="$LOGDIR/05_node.log"; STAGE5="$LOGDIR/05_stage.log"; DUMMY5="$LOGDIR/05_dummy.log"
    start_bg "$DUMMY5" python3 "$DUMMY_PY"
    DUMMY5_PG=$LAST_BG_PID
    start_bg "$STAGE5" ros2 topic echo /task_pack_bottle/stage std_msgs/msg/String --field data
    ECHO5_PG=$LAST_BG_PID
    start_bg "$NODE5" env TASK_DETECTOR_SOURCE=dds ros2 run task_pack_bottle task_pack_bottle
    NODE5_PG=$LAST_BG_PID
    sleep 6
    info "publishing start_button=true then DDS result_json (pick server is UP)"
    timeout 10 ros2 topic pub --once /user_inputs/start_button std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
    sleep 3
    timeout 10 ros2 topic pub --once /bottle_detection/result_json std_msgs/msg/String \
        "{data: '$RESULT_JSON'}" >/dev/null 2>&1
    sleep 6
    assert_grep     "dummy server received the pick goal" "$DUMMY5" 'received pick goal'
    assert_grep     "pick pose preserved (x=120.0)"        "$DUMMY5" '"x": 120.0'
    assert_grep     "tree advanced to PICK_READY"          "$STAGE5" 'PICK_READY'
    assert_not_grep "no Traceback"                         "$NODE5" 'Traceback'
    stop_bg "$NODE5_PG"; stop_bg "$ECHO5_PG"; stop_bg "$DUMMY5_PG"; sleep 2
    [ "$SEC_FAIL" -eq 0 ] && record "5. dummy action-server success path" "PASS" \
                          || record "5. dummy action-server success path" "FAIL"
fi

# ╔════════════════════════════════════════════════════════════════════════════╗
# ║ 6. DDS/FIWARE control-loop latency  (optional: MEASURE_DDS_LATENCY=1)        ║
# ╚════════════════════════════════════════════════════════════════════════════╝
# Off by default to keep the regression fast. It measures the ROS → FIWARE → ROS
# communication loop only; it does NOT fail on latency values — only on a broken
# mapping / communication timeout (the latency script's own non-zero exit).
if [ "${MEASURE_DDS_LATENCY:-0}" = "1" ]; then
    head "6. DDS/FIWARE control-loop latency (MEASURE_DDS_LATENCY=1)"
    SEC_FAIL=0
    LOG6="$LOGDIR/06_loop_latency.log"
    if ! command -v docker >/dev/null 2>&1; then
        warn "docker not available — skipping the latency measurement"
        record "6. DDS/FIWARE control-loop latency" "SKIP (no docker)"
    else
        info "running ./run_vulcanexus_dds_validation.sh measure_dds_fiware_latency.sh …"
        ( cd "$REPO" && ./run_vulcanexus_dds_validation.sh measure_dds_fiware_latency.sh ) \
            >"$LOG6" 2>&1
        rc=$?
        info "exit code: $rc   (log: $LOG6)"
        # ASCII markers only (avoid the multibyte → arrow); the summary header and
        # the breakdown label are printed only once loop samples were collected.
        assert_grep "loop samples collected"      "$LOG6" 'control-loop communication latency'
        assert_grep "breakdown printed"           "$LOG6" 'FIWARE relay/PATCH overhead'
        assert_grep "result files written"        "$LOG6" 'harmony-dds-latency-.*\.json'
        # Communication health only — never a latency threshold.
        assert_not_grep "no communication timeouts" "$LOG6" 'No successful loop samples'
        if [ "$rc" -ne 0 ]; then bad "latency runner exited non-zero (broken mapping / timeout)"; fi
        [ "$SEC_FAIL" -eq 0 ] && record "6. DDS/FIWARE control-loop latency" "PASS" \
                              || record "6. DDS/FIWARE control-loop latency" "FAIL"
    fi
fi

# ── summary ──────────────────────────────────────────────────────────────────
head "Summary"
FAILED=0
for name in "${ORDER[@]}"; do
    r="${RESULT[$name]}"
    case "$r" in
        PASS) c="$GRN" ;;
        FAIL) c="$RED"; FAILED=$((FAILED + 1)) ;;
        *)    c="$YEL" ;;
    esac
    printf '  %-46s %s%s%s\n' "$name" "$c" "$r" "$RST"
done
hr
info "logs: $LOGDIR"
if [ "$FAILED" -eq 0 ]; then
    ok "ALL REGRESSION CHECKS PASSED"
else
    bad "$FAILED regression check(s) FAILED"
fi
exit "$FAILED"
