import open3d as o3d
import numpy as np
import copy

# Load the point cloud from the .pcd file
point_cloud = o3d.io.read_point_cloud("bunny1.pcd")

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])