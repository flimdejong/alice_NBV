import open3d as o3d
import numpy as np
import copy
import os

base_dir = "/home/flimdejong/catkin_ws/PC"
# input_dir = os.path.join(base_dir, "stanford_bunny_processed")
input_dir = os.path.join(base_dir, "stanford_bunny_run_1_processed")

pcd_file = os.path.join(input_dir, "stanford_bunny_run_1_1_processed.pcd")
# pcd_file = os.path.join(input_dir, "stanford_bunny_low.pcd")

pcd = o3d.io.read_point_cloud(pcd_file)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])