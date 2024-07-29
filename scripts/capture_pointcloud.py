from camera import Camera
import time
from util.utils import *


camera = Camera(2)
time.sleep(1) # Give camera some time to load data


color_img_1, depth_img_1, color_img_2, depth_img_2 = camera.get_data_two_images()
extract_pointcloud(color_img_1, depth_img_1, color_img_2, depth_img_2, camera)

# Get the pointcloud from two sides
pcd1 = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/left_output.ply")
pcd2 = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/right_output.ply")


transformation_matrix_1 = np.loadtxt('/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_left.txt', delimiter=' ')
transformation_matrix_2 = np.loadtxt('/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_right.txt', delimiter=' ')


# Apply known transformations to align point clouds to the robot base frame
pcd1.transform(transformation_matrix_1)
pcd2.transform(transformation_matrix_2)

###########################################################################################
# Aligning pointclouds obtained from two cameras, so that there are no offsets after combine them together  
###########################################################################################
threshold = 0.02  # Distance threshold
trans_init = np.identity(4)  # Initial transformation matrix

# Apply ICP
reg_p2p = o3d.pipelines.registration.registration_icp(
    pcd1, pcd2, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())

# Transform pcd1 to align with pcd2
pcd1.transform(reg_p2p.transformation)

combined_pcd = pcd1 + pcd2

# o3d.visualization.draw_geometries([combined_pcd], window_name=f'Table Point Cloud - Iteration {1}')

    
    
    
###########################################################################################
# removes the table of the pointcloud
###########################################################################################   
    

voxel_size = 0.005  # Adjust based on your point cloud density
pcd = combined_pcd.voxel_down_sample(voxel_size)


# Iteratively remove planes (tables)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                        ransac_n=3,
                                                        num_iterations=1000)

[a, b, c, d] = plane_model

# Extract inliers (the table) and outliers (the rest of the scene)
table_cloud = pcd.select_by_index(inliers)
remaining_cloud = pcd.select_by_index(inliers, invert=True)

# Visualize the current iteration's table point cloud
table_cloud.paint_uniform_color([1, 0, 0])  # Color the table red
o3d.visualization.draw_geometries([table_cloud], window_name=f'Table Point Cloud - Iteration {1}')

# Visualize the remaining point cloud (without the table)
o3d.visualization.draw_geometries([remaining_cloud], window_name='Point Cloud without Table')