import open3d as o3d
import numpy as np

# Load the point cloud
pcd = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/1705634038453106.pcd")

# Convert to numpy array
points = np.asarray(pcd.points)

# Calculate the median of the x-coordinates
median_y = np.median(points[:, 1])

# Split the point cloud
left_half_points = points[points[:, 1] < median_y]
right_half_points = points[points[:, 1] >= median_y]

# Create Open3D point cloud objects for each half
left_half_pcd = o3d.geometry.PointCloud()
left_half_pcd.points = o3d.utility.Vector3dVector(left_half_points)

right_half_pcd = o3d.geometry.PointCloud()
right_half_pcd.points = o3d.utility.Vector3dVector(right_half_points)

# # Visualize the two halves
# o3d.visualization.draw_geometries([left_half_pcd], window_name="Left Half")
# o3d.visualization.draw_geometries([right_half_pcd], window_name="Right Half")



