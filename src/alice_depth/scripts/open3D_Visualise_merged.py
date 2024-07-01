#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import copy
import os

base_dir = "/home/flimdejong/catkin_ws/PC"
input_dir = os.path.join(base_dir, "stanford_bunny_run_2_merged")

pcd_file = os.path.join(input_dir, "stanford_bunny_run_1_4_merged.pcd")
# pcd_file = os.path.join(input_dir, "stanford_bunny_low.pcd")

pcd = o3d.io.read_point_cloud(pcd_file)

# Final visualization with black background
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])  # Set background color to black
vis.run()
vis.destroy_window()


