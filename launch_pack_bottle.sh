#!/usr/bin/env bash
# Run with: ./launch_pack_bottle.sh --camera=2
# Default camera is 0 if not specified.

CAMERA=0
for arg in "$@"; do
    case "$arg" in
        --camera=*) CAMERA="${arg#--camera=}" ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"

launch() {
    local label="$1" command="$2"
    local tmp; tmp="$(mktemp /tmp/pack_bottle_XXXXXX.sh)"
    printf '#!/usr/bin/env bash\ncd "%s"\n%s\nexec bash\n' "$SCRIPT_DIR" "$command" > "$tmp"
    gnome-terminal --tab --title="$label" -- bash "$tmp"
}

# Integrated demonstrator: standard Orion (NGSI-v2) + all components + the NGSI-v2
# node bridge. The standalone fiware_bridge module defaults to dds, but this
# all-in-one demo is NGSI-v2, so the ROS tab pins BRIDGE_BACKEND=node. (The DDS
# path is a separate, bridge-only procedure — see fiware_bridge/dds/README.md.)
launch "Fiware Docker"           "cd fiware-analytics-docker && docker compose up -d"
launch "React Dashboard"         "cd react-dashboard && . ./run.sh"
launch "AI Bottle Detector"      "source torch_venv/bin/activate && cd ai-bottle-detector-fiware && . ./run.sh"
launch "Hand Gesture Detector"   "sleep 5 && source venv/bin/activate && cd gesture-commands-fiware && python gesture-commands-fiware.py --camera $CAMERA"
launch "Voice Commands Detector" "sleep 5 && source venv/bin/activate && cd voice-commands-fiware && python voice-commands-fiware.py --fiware"
launch "ROS 2"                   "sleep 20 && export BRIDGE_BACKEND=node && cd ros2-xarm-pack-bottle && . ./run.sh"