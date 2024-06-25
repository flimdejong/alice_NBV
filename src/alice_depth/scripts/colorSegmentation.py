#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import os

base_dir = "/home/flimdejong/catkin_ws/PC"
input_dir = os.path.join(base_dir, "stanford_bunny")
pcd_file = os.path.join(base_dir, "stanford_bunny.pcd")

pcd = o3d.io.read_point_cloud(pcd_file)

plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)

# Remove background points from the point cloud
removed_points = pcd.select_by_index(inliers)

# The remaining PC
white_pcd = pcd.select_by_index(inliers, invert = True)


################

# Get colors as a numpy array
colors = np.asarray(white_pcd.colors)

# Define color threshold for white
white_threshold = 0.75  # This value can be adjusted based on your needs

# Check if all color channels are above the threshold
is_white = np.all(colors > white_threshold, axis=1)

# Create a new point cloud with only the white points
white_pcd = white_pcd.select_by_index(np.where(is_white)[0])

# Applying plane segmentation using RANSAC (for background removal)
# To find the plane with the largest support in the point cloud, we can use segment_plane. 
# The method has three arguments: distance_threshold defines the maximum distance a point can have to an estimated plane to be considered an inlier
# ransac_n defines the number of points that are randomly sampled to estimate a plane, and num_iterations
# Applying plane segmentation using RANSAC (for floor removal)


# Statistical outlier removal
# nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
# std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud. 
# The lower this number the more aggressive the filter will be.

cl, ind = white_pcd.remove_statistical_outlier(nb_neighbors=35,
                                                    std_ratio=0.1)

# Remove floor points from the point cloud, ind is the indices of points which are NOT outliers
removed_points = white_pcd.select_by_index(ind, invert = True)

# The remaining PC
remaining_cloud = white_pcd.select_by_index(ind)

# Visualize the result with black background
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(remaining_cloud)

# Set background color to black
opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])  # RGB for black

vis.run()
vis.destroy_window()