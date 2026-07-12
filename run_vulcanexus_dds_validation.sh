#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_vulcanexus_dds_validation.sh
#
# Runs ./validate_dds_native_demo.sh inside the official Vulcanexus Jazzy Docker
# image (eprosima/vulcanexus:jazzy-desktop). Vulcanexus is the recommended ARISE
# validation runtime: it reliably propagates the std_msgs::msg::dds_::String_
# TypeObject, so the Orion-LD NGSI-LD values come back REAL instead of
# "uninitialized" (which is what plain ROS 2 Jazzy tends to produce).
#
# The container uses host networking + host IPC so ROS 2 / Fast DDS discovery and
# the Orion-LD broker (on localhost:1026) are reachable from inside.
#
# The Vulcanexus image has no Docker CLI, so this wrapper makes sure the DDS
# Orion-LD broker is already running ON THE HOST before launching the container.
# The validation script then simply finds it up — its behaviour is NOT modified.
# No NGSI-v2 component is started (only the NGSI-LD DDS broker + the ros2 backends).
# ─────────────────────────────────────────────────────────────────────────────

set -u

REPO="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
IMAGE="${VULCANEXUS_IMAGE:-eprosima/vulcanexus:jazzy-desktop}"
DDS_DIR="$REPO/ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds"
ORION="${ORION:-http://localhost:1026}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Which validation to run inside the container. Defaults to the DDS-native demo
# validation; pass another top-level script, e.g.:
#     ./run_vulcanexus_dds_validation.sh validate_dds_command_flow.sh
#     ./run_vulcanexus_dds_validation.sh measure_dds_fiware_latency.sh
TARGET="${1:-validate_dds_native_demo.sh}"
if [ ! -f "$REPO/$TARGET" ]; then
    echo "[host] ERROR: validation script '$TARGET' not found in $REPO" >&2
    exit 1
fi

broker_up() { curl -s --max-time 3 "$ORION/version" 2>/dev/null | grep -qi 'orionld'; }

# ── 1. Ensure the DDS Orion-LD broker is up on the host ──────────────────────
echo "[host] checking DDS Orion-LD broker on $ORION ..."
if broker_up; then
    echo "[host] DDS Orion-LD broker already running."
else
    ver="$(curl -s --max-time 3 "$ORION/version" 2>/dev/null)"
    if printf '%s' "$ver" | grep -qi 'orion'; then
        echo "[host] ERROR: port 1026 is served by a NON-LD Orion (NGSI-v2 stack?)." >&2
        echo "[host]        Stop it first — DDS validation needs the Orion-LD DDS broker." >&2
        exit 1
    fi
    echo "[host] starting DDS Orion-LD broker (docker compose up -d) ..."
    ( cd "$DDS_DIR" && docker compose -f docker-compose.dds.yml up -d ) || exit 1
    for _ in $(seq 1 20); do broker_up && break; sleep 1; done
    if ! broker_up; then
        echo "[host] ERROR: DDS broker did not come up on $ORION" >&2
        exit 1
    fi
    echo "[host] DDS Orion-LD broker up."
fi

# ── 2. Make sure the Vulcanexus image is available ───────────────────────────
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "[host] image $IMAGE not present locally — Docker will pull it (this is large) ..."
fi

# Allocate a TTY only when running interactively (keeps it CI-friendly).
TTY_FLAG=""
[ -t 1 ] && TTY_FLAG="-t"

# ── 3. Run the validation inside Vulcanexus ──────────────────────────────────
# Mount the repo at the SAME absolute path so realpath-based paths resolve, use
# host networking + IPC for DDS/Orion discovery, and pin the ROS domain.
echo "[host] launching $IMAGE (host net + host ipc) — running ./$TARGET ..."
exec docker run --rm -i $TTY_FLAG \
    --net=host \
    --ipc=host \
    -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    -e "ORION=$ORION" \
    -e "TARGET=$TARGET" \
    -e MEASURE_DDS_LATENCY \
    -e DDS_LATENCY_WARMUP \
    -e DDS_LATENCY_SAMPLES \
    -e DDS_LATENCY_TIMEOUT_SEC \
    -e DDS_LATENCY_POLL_INTERVAL_MS \
    -e DDS_LATENCY_OUTDIR \
    -v "$REPO:$REPO" \
    -w "$REPO" \
    "$IMAGE" \
    bash -lc '
        source /opt/vulcanexus/jazzy/setup.bash
        WS="ros2-xarm-pack-bottle/ros2_ws"
        if [ ! -f "$WS/install/setup.bash" ]; then
            echo "[container] $WS not built — running colcon build ..."
            ( cd "$WS" && colcon build ) \
                || echo "[container] WARN: colcon build failed; continuing (std_msgs-only validation may still work)."
        fi
        exec ./"$TARGET"
    '
