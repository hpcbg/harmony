#!/usr/bin/env bash
# Bottle detector — two backends, selected via AI_BACKEND:
#
#   AI_BACKEND=fiware_v2  (default)  the full NGSI-v2 detector service (FastAPI),
#                                    unchanged:  uvicorn main:app on :22001.
#                                        ./run.sh
#                                        AI_BACKEND=fiware_v2 ./run.sh
#
#   AI_BACKEND=ros2                   DDS-native experimental first step: a minimal
#                                    ROS 2 node (ros2_backend.py) that subscribes to
#                                    /bottle_detection/command and publishes simple
#                                    status to /bottle_detection/status_json. No /v2
#                                    calls, no FastAPI/OpenCV/PyTorch/model weights.
#                                        AI_BACKEND=ros2 ./run.sh
#
# No-camera self-test (ros2 backend): set TEST_DETECTION_COMMAND to inject a command
# without a camera, GPU, PyTorch, model weights or FIWARE NGSI-v2, e.g.
#       AI_BACKEND=ros2 TEST_DETECTION_COMMAND=START ./run.sh
#
# Real perception (ros2 backend) — same DDS interface, real detector pipeline:
#       AI_BACKEND=ros2 AI_DETECTION_MODE=real ./run.sh
#   Real mode needs torch/torchvision/cv2 + model weights; it uses the detector venv
#   python (torch_venv), which still imports rclpy from the sourced ROS PYTHONPATH.
#   Override the camera index with AI_CAMERA=<n> (defaults to config/config.json).
#
# For the ros2 backend a ROS 2 environment is required. This script sources one
# automatically (Vulcanexus Jazzy first — the ARISE-compliant choice — then plain
# ROS 2 Jazzy for local debugging) if rclpy is not already importable.

BACKEND="${AI_BACKEND:-fiware_v2}"
MODE="${AI_DETECTION_MODE:-stub}"
HERE="$(cd "$(dirname "$0")" && pwd)"

# Prefer the venv's `python`; fall back to system `python3` (e.g. after sourcing
# ROS in a plain shell where only python3 exists).
PY="$(command -v python || command -v python3)"

# Real detection needs torch/torchvision/cv2 (the detector venv) AND rclpy (ROS).
# The detector venv python (3.12) imports rclpy from the sourced ROS PYTHONPATH, so
# one interpreter can do both. Use it for AI_DETECTION_MODE=real.
TORCH_PY="$HERE/../torch_venv/bin/python"
if [ "$BACKEND" = "ros2" ] && [ "$MODE" = "real" ]; then
    if [ -x "$TORCH_PY" ]; then
        PY="$TORCH_PY"
        echo "[run.sh] real detection mode → detector venv python ($PY)"
    else
        echo "[run.sh] WARNING: AI_DETECTION_MODE=real but detector venv not found at"
        echo "[run.sh]          $TORCH_PY — torch/cv2 may be unimportable; detection will report FAILED."
    fi
fi

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
        # Keep the deliberately-chosen detector venv python in real mode (rclpy now
        # resolves from the sourced ROS PYTHONPATH); otherwise pick up the ROS python.
        if [ "$MODE" != "real" ]; then
            PY="$(command -v python || command -v python3)"
        fi
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

    exec "$PY" ros2_backend.py
else
    exec uvicorn main:app --host 0.0.0.0 --port 22001
fi
