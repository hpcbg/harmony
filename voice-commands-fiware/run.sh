#!/usr/bin/env bash
# Voice command publisher — two backends, selected via the VOICE_BACKEND env var:
#
#   VOICE_BACKEND=fiware_v2   (default)  publish NGSI-v2 entities to Orion.
#                                        This is the integrated node-bridge demo.
#                                            ./run.sh
#                                            VOICE_BACKEND=fiware_v2 ./run.sh
#
#   VOICE_BACKEND=ros2                    DDS-native experimental path: publish
#                                        std_msgs/String on /user_inputs/voice_command.
#                                        Orion-LD (-wip dds) maps that ROS 2/DDS topic
#                                        to NGSI-LD, so no /v2 calls and no HTTP 501.
#                                            VOICE_BACKEND=ros2 ./run.sh
#
# No-mic self-test (any backend): set TEST_KEYWORD to publish without audio, e.g.
#       VOICE_BACKEND=ros2 TEST_KEYWORD=PICK ./run.sh
#       VOICE_BACKEND=ros2 TEST_KEYWORD="PICK STOP GO" ./run.sh
#
# For the ros2 backend a ROS 2 environment is required. This script sources one
# automatically (Vulcanexus Jazzy first — the ARISE-compliant choice — then plain
# ROS 2 Jazzy for local debugging) if rclpy is not already importable.

BACKEND="${VOICE_BACKEND:-fiware_v2}"

# Prefer the venv's `python`; fall back to system `python3` (e.g. after sourcing
# ROS in a plain shell where only python3 exists).
PY="$(command -v python || command -v python3)"

ros2_ready() { "$PY" -c "import rclpy, std_msgs.msg" 2>/dev/null; }

if [ "$BACKEND" = "ros2" ]; then
    if ! ros2_ready; then
        for ros_setup in /opt/vulcanexus/jazzy/setup.bash /opt/ros/jazzy/setup.bash; do
            if [ -f "$ros_setup" ]; then
                echo "[run.sh] sourcing $ros_setup"
                # shellcheck disable=SC1090
                source "$ros_setup"
                case "$ros_setup" in
                    */vulcanexus/*) ;;  # ARISE-compliant
                    *) echo "[run.sh] NOTE: plain ROS 2 Jazzy is for local debugging; ARISE DDS validation should use Vulcanexus Jazzy." ;;
                esac
                break
            fi
        done
        # Overlay the project workspace if it has been built (custom interfaces).
        WS_SETUP="../ros2-xarm-pack-bottle/ros2_ws/install/setup.bash"
        # shellcheck disable=SC1090
        [ -f "$WS_SETUP" ] && source "$WS_SETUP"
        PY="$(command -v python || command -v python3)"
    fi

    if ! ros2_ready; then
        cat <<'EOF'

ROS 2 / Vulcanexus Python packages are not importable.

For ARISE-compliant DDS validation, run this module in a Vulcanexus Jazzy environment:

  source /opt/vulcanexus/jazzy/setup.bash
  source ../ros2-xarm-pack-bottle/ros2_ws/install/setup.bash

Plain ROS 2 Jazzy may be useful for local debugging, but ARISE DDS validation should be done with Vulcanexus.
EOF
        exit 1
    fi

    exec "$PY" voice-commands-fiware.py --keywords
else
    exec "$PY" voice-commands-fiware.py --fiware
fi
