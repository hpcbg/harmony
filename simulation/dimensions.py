import isaacsim.core.utils.bounds as bounds_utils
import numpy as np

prim_path = "/World/Bottle"
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

prim_path = "/World/Bottle"
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
