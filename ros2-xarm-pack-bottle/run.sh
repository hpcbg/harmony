cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Bridge backend defaults to 'node' (NGSI-v2, integrated demonstrator).
# Opt into the standalone Orion-LD DDS bridge via: BRIDGE_BACKEND=dds ./run.sh
# (requires the DDS broker up and replaces the standard Orion stack — see dds/README.md).
ros2 launch ./launch/complete.launch.py bridge_backend:=${BRIDGE_BACKEND:-node}
cd ..