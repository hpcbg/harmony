# 7x2 cillinder bottle for hamony


from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdGeom
import numpy as np
from scipy.spatial.transform import Rotation

def get_object_pose_from_isaac(prim_path):
        """Get object pose from Isaac Sim"""
        stage = get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        
        if not prim.IsValid():
            self.get_logger().warn(f"Prim {prim_path} not found")
            return None
            
        xform = UsdGeom.Xformable(prim)
        transform_matrix = xform.ComputeLocalToWorldTransform(0)
        
        # Extract position
        position = transform_matrix.ExtractTranslation()
        
        # Extract rotation as quaternion
        rotation_matrix = transform_matrix.ExtractRotationMatrix()
        rotation = Rotation.from_matrix(np.array(rotation_matrix))
        quat = rotation.as_quat()  # [x, y, z, w]
        
        return {
            'position': [position[0], position[1], position[2]],
            'orientation': [quat[0], quat[1], quat[2], quat[3]]
        }

prim_path = "/World/packing_table"
pose = get_object_pose_from_isaac(prim_path)
print(pose)



import isaacsim.core.utils.bounds as bounds_utils
import numpy as np

prim_path = "/World/packing_table"
cache = bounds_utils.create_bbox_cache()

# Compute the AABB
aabb = bounds_utils.compute_aabb(cache, prim_path)

# Extract min and max corners
min_corner = np.array(aabb[0:3])
max_corner = np.array(aabb[3:6])

# Calculate dimensions
dimensions = max_corner - min_corner

print("--- AABB Method (Not Recommended) ---")
print(f"Raw Output: {aabb}")
print(f"Dimensions (L, W, H): {dimensions}")

# --- Assuming you used Method 2 (OBB) ---
# dimensions = [diameter, diameter, height]

# Assuming the Z-axis (index 2) is the height
height = dimensions[2]
diameter = dimensions[0]  # Or dimensions[1]
radius = diameter / 2.0

print("\n--- MoveIt 2 Cylinder Dimensions from AABB---")
print(f"Height: {height}")
print(f"Radius: {radius}")

# You would then use these in your C++ or Python code:
# primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
# primitive.dimensions = { height, radius };

import isaacsim.core.utils.bounds as bounds_utils
import numpy as np

prim_path = "/World/packing_table"
cache = bounds_utils.create_bbox_cache()

# Compute the OBB
centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path)

# Calculate dimensions
dimensions = half_extent * 2

print("--- OBB Method (Recommended) ---")
print(f"Raw Half-Extents: {half_extent}")
print(f"Dimensions (L, W, H): {dimensions}")


# --- Assuming you used Method 2 (OBB) ---
# dimensions = [diameter, diameter, height]

# Assuming the Z-axis (index 2) is the height
height = dimensions[2]
diameter = dimensions[0]  # Or dimensions[1]
radius = diameter / 2.0

print("\n--- MoveIt 2 Cylinder Dimensions OBB ---")
print(f"Height: {height}")
print(f"Radius: {radius}")

# You would then use these in your C++ or Python code:
# primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
# primitive.dimensions = { height, radius };

h 0.09217
r 0.01908

