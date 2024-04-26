import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/1702357444278407.pcd")

o3d.visualization.draw_geometries([pcd])
