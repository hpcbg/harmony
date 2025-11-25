import numpy as np
from scipy.spatial.transform import Rotation

# --- 1. Your Input Pose from Isaac Sim (Y-Up) ---
# From your screenshot: image_393be2.png
pos_isaac = np.array([-1.05589, -0.01194, 0.99405])
# Isaac Sim 'Orien' (X,Y,Z) is (Roll, Pitch, Yaw) in degrees
rot_isaac_euler_deg = np.array([0.0, 0.0, 0.0])

# --- 2. Define the Y-Up to Z-Up Conversion ---
# This is a fixed rotation of -90 degrees around the X-axis
rot_yup_to_zup = Rotation.from_euler('x', -90, degrees=True)

# --- 3. Convert the Pose ---
# Convert the Isaac rotation to a Rotation object
rot_isaac = Rotation.from_euler('xyz', rot_isaac_euler_deg, degrees=True)

# Convert position:
# ROS X = Isaac X
# ROS Y = -Isaac Z
# ROS Z = Isaac Y
pos_ros = np.array([
    pos_isaac[0],
    -pos_isaac[2],
    pos_isaac[1]
])

# Convert rotation:
# The final rotation in ROS is (Conversion) * (Isaac Rotation)
rot_ros = rot_yup_to_zup * rot_isaac

# --- 4. Get ROS 2 Output for static_transform_publisher ---
# The command line tool expects yaw, pitch, roll
euler_ros_rad = rot_ros.as_euler('zyx', degrees=False) # ZYX order for yaw, pitch, roll
yaw = euler_ros_rad[0]
pitch = euler_ros_rad[1]
roll = euler_ros_rad[2]

# --- 5. Print the results ---
print("--- Isaac Sim (Y-Up) Pose ---")
print(f"Position (x, y, z): {pos_isaac}")
print(f"Rotation (roll, pitch, yaw) deg: {rot_isaac_euler_deg}")

print("\n--- ROS 2 (Z-Up) Pose ---")
print("Copy these values into your static_transform_publisher 'arguments':")
print(f"\n# arguments: [x, y, z, yaw, pitch, roll, parent, child]")
print(f"arguments: [")
print(f"    '{pos_ros[0]}',")
print(f"    '{pos_ros[1]}',")
print(f"    '{pos_ros[2]}',")
print(f"    '{yaw}',")
print(f"    '{pitch}',")
print(f"    '{roll}',")
print(f"    'World',")
print(f"    'world'")
print(f"]")