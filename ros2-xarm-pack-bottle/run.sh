cd ros2_ws
# ARISE fixes Fast DDS as the middleware: prefer Vulcanexus Jazzy, fall back to
# a standard ROS 2 Jazzy install.
if [ -f /opt/vulcanexus/jazzy/setup.bash ]; then
    source /opt/vulcanexus/jazzy/setup.bash
else
    source /opt/ros/jazzy/setup.bash
fi
source install/setup.bash
# Bridge backend defaults to 'node' (NGSI-v2, integrated demonstrator).
# Opt into the standalone Orion-LD DDS bridge via: BRIDGE_BACKEND=dds ./run.sh
# (requires the DDS broker up and replaces the standard Orion stack — see dds/README.md).
ros2 launch ./launch/complete.launch.py bridge_backend:=${BRIDGE_BACKEND:-node}
cd ..