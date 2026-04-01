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

launch "Fiware Docker"           "cd fiware-docker && docker compose up -d"
launch "React Dashboard"         "cd react-dashboard && . ./run.sh"
launch "AI Bottle Detector"      "source torch_env/bin/activate && cd ai-bottle-detector && . ./run.sh"
launch "Hand Gesture Detector"   "sleep 5 && source venv/bin/activate && cd gesture-commands-fiware && python gesture-commands-fiware.py --camera $CAMERA"
launch "Voice Commands Detector" "sleep 5 && source venv/bin/activate && cd voice-commands-fiware && python voice-commands-fiware.py --fiware"
launch "ROS 2"                   "sleep 20 && cd ros2-fiware-xarm && . ./run.sh"