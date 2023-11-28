import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/good_one.pcd")

o3d.visualization.draw_geometries([pcd])
