#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# launch_pack_bottle_dds.sh
#
# DDS-native counterpart of launch_pack_bottle.sh. Instead of the NGSI-v2 Orion
# stack + custom node bridge, this brings up the ARISE **DDS-broker-only** path:
#
#   • FIWARE side  : the Orion-LD DDS broker (docker-compose.dds.yml, -wip dds
#                    -mongocOnly, NGSI-LD only) — NOT the fiware-analytics stack.
#   • Voice/Gesture: their DDS-native `ros2` backends (VOICE_BACKEND=ros2 /
#                    GESTURE_BACKEND=ros2) — publish std_msgs to ROS 2/DDS.
#   • AI detector  : AI_BACKEND=ros2 AI_DETECTION_MODE=real — real Faster-R-CNN
#                    perception behind the same four DDS topics (uses torch_venv).
#   • ROS 2        : BRIDGE_BACKEND=dds (the default) — no node runs; Orion-LD
#                    bridges the topics directly.
#
# The React dashboard is deliberately **NOT** started: it reads NGSI-v2, which
# the -mongocOnly DDS broker returns HTTP 501 for. Everything else in the demo
# has been validated end-to-end on this DDS path (see run_dds_regression_tests.sh).
#
# Requirements: a Vulcanexus Jazzy ROS side (reliable DDS TypeObject propagation)
# and Docker. The DDS broker and the standard Orion stack both bind host port
# 1026 — do NOT run the fiware-analytics stack (or launch_pack_bottle.sh) at the
# same time.
#
# Run with: ./launch_pack_bottle_dds.sh --camera=2
#   --camera=N   gesture camera device ID (default 0).
#   AI_DETECTION_MODE=stub ./launch_pack_bottle_dds.sh   # no camera/weights AI path
# ─────────────────────────────────────────────────────────────────────────────

CAMERA=0
for arg in "$@"; do
    case "$arg" in
        --camera=*) CAMERA="${arg#--camera=}" ;;
    esac
done

AI_MODE="${AI_DETECTION_MODE:-real}"
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"

# Source a ROS 2 env (Vulcanexus first — the ARISE-compliant choice — then plain
# Jazzy) plus the project workspace, so the venv python that runs the gesture/voice
# modules can import rclpy from the sourced PYTHONPATH (the same single-interpreter
# approach the AI detector's torch_venv uses).
ROS_SRC='{ [ -f /opt/vulcanexus/jazzy/setup.bash ] && source /opt/vulcanexus/jazzy/setup.bash || source /opt/ros/jazzy/setup.bash; source ros2-xarm-pack-bottle/ros2_ws/install/setup.bash 2>/dev/null || true; }'

# Warn (do not block) if a non-LD Orion is already holding port 1026 — the DDS
# broker cannot bind it and the demo would silently talk to the wrong broker.
ver="$(curl -s --max-time 3 http://localhost:1026/version 2>/dev/null)"
if printf '%s' "$ver" | grep -qi 'orion' && ! printf '%s' "$ver" | grep -qi 'orionld'; then
    echo "WARNING: a non-LD Orion (NGSI-v2) is already on :1026 — stop it first;"
    echo "         the DDS Orion-LD broker needs that port. Continuing anyway..."
fi

launch() {
    local label="$1" command="$2"
    local tmp; tmp="$(mktemp /tmp/pack_bottle_dds_XXXXXX.sh)"
    printf '#!/usr/bin/env bash\ncd "%s"\n%s\nexec bash\n' "$SCRIPT_DIR" "$command" > "$tmp"
    gnome-terminal --tab --title="$label" -- bash "$tmp"
}

# 1. FIWARE side: the Orion-LD DDS broker (NGSI-LD only). Replaces the NGSI-v2
#    analytics stack; both bind :1026, so only one may run at a time.
launch "FIWARE DDS Broker"           "cd ros2-xarm-pack-bottle/ros2_ws/src/fiware_bridge/dds && docker compose -f docker-compose.dds.yml up -d && echo 'DDS Orion-LD broker up on :1026 (NGSI-LD only).'"

# (React dashboard intentionally skipped — NGSI-v2 only; the DDS broker returns 501.)

# 2. AI detector — DDS-native real perception on the four bottle_detection topics.
launch "AI Bottle Detector (DDS)"    "sleep 5 && cd ai-bottle-detector-fiware && AI_BACKEND=ros2 AI_DETECTION_MODE=$AI_MODE ./run.sh"

# 3. Gesture — DDS-native publisher (needs mediapipe from venv + rclpy from ROS).
launch "Hand Gesture Detector (DDS)" "sleep 8 && source venv/bin/activate && $ROS_SRC && cd gesture-commands-fiware && GESTURE_BACKEND=ros2 python gesture-commands-fiware.py --camera $CAMERA --no-gui"

# 4. Voice — DDS-native publisher (needs vosk from venv + rclpy from ROS).
launch "Voice Commands Detector (DDS)" "sleep 8 && source venv/bin/activate && $ROS_SRC && cd voice-commands-fiware && VOICE_BACKEND=ros2 python voice-commands-fiware.py --keywords"

# 5. ROS 2 task + xArm control over the DDS bridge backend (the default).
launch "ROS 2 (DDS bridge)"          "sleep 20 && export BRIDGE_BACKEND=dds && cd ros2-xarm-pack-bottle && . ./run.sh"
