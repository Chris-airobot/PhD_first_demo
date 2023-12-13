import open3d as o3d
import numpy as np

# Load your point cloud
pcd = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/1702357444278407.pcd")

# Convert Open3D point cloud to NumPy array
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

# Define the color range you want to keep (for example, green color)
target_color = np.array([200, 163, 120])  
color_threshold = 0.1  # Adjust this threshold based on your color matching criteria

# Calculate color differences
color_diff = np.linalg.norm(colors - target_color, axis=1)

# Create a mask for points with color close to the target color
mask = color_diff < color_threshold

# Extract the points matching the mask
filtered_points = points[mask]
filtered_colors = colors[mask]

# Create a new Open3D point cloud with the filtered points and colors
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

# Visualize the original and filtered point clouds
o3d.visualization.draw_geometries([pcd, filtered_pcd])
