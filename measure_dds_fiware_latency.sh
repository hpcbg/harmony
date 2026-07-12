#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# measure_dds_fiware_latency.sh
#
# Measures the DDS-native FIWARE *control-loop communication latency*:
#
#     ROS 2 publisher (/user_inputs/gesture_command)
#       → Orion-LD DDS enabler → NGSI-LD GestureDetector:operator-1
#       → minimal TEST RELAY: PATCH BottleDetectionJob:processor-01.command
#       → Orion-LD DDS enabler → ROS 2 (/bottle_detection/command)
#       → ROS 2 subscriber
#
# The primary reported value is the ROS → FIWARE → ROS loop latency. It excludes
# camera processing, AI inference, gesture recognition, robot motion, dashboard
# rendering, and any application logic — the only step between the two DDS hops is
# the minimal test relay (documented as "test relay overhead", not perception or
# robot logic).
#
# It needs NO camera, NO AI real mode, NO gesture real mode, NO robot hardware.
# It refuses to run against a NGSI-v2-only Orion.
#
# Recommended runtime: Vulcanexus Jazzy (reliable TypeObject propagation). Run it
# inside the Vulcanexus image with:
#     ./run_vulcanexus_dds_validation.sh measure_dds_fiware_latency.sh
#
# Environment overrides:
#     DDS_LATENCY_WARMUP           (default 3)
#     DDS_LATENCY_SAMPLES          (default 20)
#     DDS_LATENCY_TIMEOUT_SEC      (default 5)
#     DDS_LATENCY_POLL_INTERVAL_MS (default 5)
#     DDS_LATENCY_OUTDIR           (default /tmp; point at a mounted path when
#                                   running via the Vulcanexus Docker wrapper,
#                                   whose container /tmp is ephemeral)
#     ORION                        (default http://localhost:1026)
#
# Outputs: <DDS_LATENCY_OUTDIR>/harmony-dds-latency-<timestamp>.json and .csv
# ─────────────────────────────────────────────────────────────────────────────

set -u

SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
WS="$SCRIPT_DIR/ros2-xarm-pack-bottle/ros2_ws"
DDS_DIR="$WS/src/fiware_bridge/dds"
HELPER="$SCRIPT_DIR/scripts/measure_dds_fiware_latency.py"
ORION="${ORION:-http://localhost:1026}"

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

# ── cleanup: kill any stray probe process on exit ─────────────────────────────
cleanup() {
    pkill -f "measure_dds_fiware_latency.py" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

head "DDS/FIWARE control-loop latency"
info "helper   : $HELPER"
info "Orion-LD : $ORION"

if [ ! -f "$HELPER" ]; then
    fail "helper not found: $HELPER"
    exit 1
fi

# ── source a ROS 2 environment (Vulcanexus first, plain Jazzy fallback) ───────
ROS_SETUP=""
set +u
for s in /opt/vulcanexus/jazzy/setup.bash /opt/ros/jazzy/setup.bash; do
    if [ -f "$s" ]; then
        # shellcheck disable=SC1090
        source "$s"; ROS_SETUP="$s"; break
    fi
done
# shellcheck disable=SC1090
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
if [ -z "$ROS_SETUP" ]; then
    fail "no ROS 2 environment found (looked for Vulcanexus Jazzy and ROS 2 Jazzy)."
    info "source /opt/vulcanexus/jazzy/setup.bash before running this measurement."
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
case "$ROS_SETUP" in
    */vulcanexus/*) pass "Vulcanexus detected — reliable DDS type propagation expected." ;;
    *)              warn "Plain ROS 2 Jazzy: values may stay \"uninitialized\"; use Vulcanexus for ARISE." ;;
esac
info "ROS env       : $ROS_SETUP"
info "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"

# ── refuse the wrong broker (NGSI-v2-only stack) ──────────────────────────────
head "Broker check"
ver="$(curl -s --max-time 3 "$ORION/version" 2>/dev/null)"
if printf '%s' "$ver" | grep -qi 'orionld'; then
    pass "DDS Orion-LD broker confirmed on $ORION"
elif printf '%s' "$ver" | grep -qi 'orion'; then
    fail "Port 1026 is served by a NON-LD Orion (NGSI-v2 stack?). Refusing to run."
    info "This measurement requires the DDS/NGSI-LD Orion-LD broker."
    exit 1
else
    warn "DDS broker not reachable — starting it (docker compose up -d) …"
    if command -v docker >/dev/null 2>&1; then
        ( cd "$DDS_DIR" && docker compose -f docker-compose.dds.yml up -d ) >/dev/null 2>&1
        for i in $(seq 1 20); do
            curl -s --max-time 2 "$ORION/version" 2>/dev/null | grep -qi 'orionld' \
                && { pass "DDS Orion-LD broker up after ${i}s"; break; }
            sleep 1
        done
    fi
    curl -s --max-time 2 "$ORION/version" 2>/dev/null | grep -qi 'orionld' \
        || { fail "DDS broker did not come up on $ORION"; exit 1; }
fi

# ── run the measurement ───────────────────────────────────────────────────────
PY="$(command -v python3 || command -v python)"
export ORION
exec "$PY" "$HELPER"
