cd ros2_ws
# ARISE fixes Fast DDS as the middleware: prefer Vulcanexus Jazzy, fall back to
# a standard ROS 2 Jazzy install.
if [ -f /opt/vulcanexus/jazzy/setup.bash ]; then
    source /opt/vulcanexus/jazzy/setup.bash
else
    source /opt/ros/jazzy/setup.bash
fi
source install/setup.bash
# Bridge backend defaults to 'dds' (DDS-broker-only architecture): no node runs,
# the Orion-LD DDS broker bridges directly. Requires the DDS broker up
# (docker-compose.dds.yml) and a Vulcanexus ROS side. The DDS broker is NGSI-LD
# only — the NGSI-v2 components (voice/gesture/AI/dashboard) do not run against it.
# For the NGSI-v2 node bridge + standard Orion stack: BRIDGE_BACKEND=node ./run.sh
ros2 launch ./launch/complete.launch.py bridge_backend:=${BRIDGE_BACKEND:-dds}
cd ..