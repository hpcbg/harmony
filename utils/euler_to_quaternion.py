import numpy as np
from scipy.spatial.transform import Rotation

# Input Euler angles in degrees (Roll, Pitch, Yaw)
euler_deg = [-90.414, -0.225, -0.114]

# Create rotation object (XYZ order is standard for Isaac/ROS roll-pitch-yaw)
r = Rotation.from_euler('xyz', euler_deg, degrees=True)

# Convert to Quaternion [x, y, z, w]
quat = r.as_quat()

print(f"x: {quat[0]:.6f}")
print(f"y: {quat[1]:.6f}")
print(f"z: {quat[2]:.6f}")
print(f"w: {quat[3]:.6f}")