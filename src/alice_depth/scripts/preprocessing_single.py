import open3d as o3d
import numpy as np
import os

base_dir = "/home/flimdejong/catkin_ws/PC"  # Replace with the appropriate base directory
input_dir = os.path.join(base_dir, "stanford_bunny_run_1")
# output_dir = os.path.join(base_dir, "stanford_bunny_processed")

# Load the point cloud from the .pcd file
pcd_file = os.path.join(input_dir, "stanford_bunny_run_1_1.pcd")
pcd = o3d.io.read_point_cloud(pcd_file)

# Print the point cloud details
print(pcd)

# 0.005 is too big. 0.003 is kinda fine.
pcd = pcd.voxel_down_sample(voxel_size=0.001)

# Applying plane segmentation using RANSAC (for background removal)
# To find the plane with the largest support in the point cloud, we can use segment_plane. 
# The method has three arguments: distance_threshold defines the maximum distance a point can have to an estimated plane to be considered an inlier
# ransac_n defines the number of points that are randomly sampled to estimate a plane, and num_iterations
# Applying plane segmentation using RANSAC (for floor removal)

plane_model, inliers = pcd.segment_plane(distance_threshold=0.04, ransac_n=3, num_iterations=2000)

# Remove floor points from the point cloud
removed_points = pcd.select_by_index(inliers)

# The remaining PC
remaining_cloud = pcd.select_by_index(inliers, invert = True)

# Assign a specific color to the removed points (e.g., red)
removed_points.paint_uniform_color([1, 0, 0])  # Red color

# Visualize the original point cloud and the removed points
o3d.visualization.draw_geometries([remaining_cloud, removed_points])


###############
"""" Carry out color segmentation """
###############

# Get colors as a numpy array
colors = np.asarray(remaining_cloud.colors)

# Define color threshold for white
white_threshold = 0.75

# Check if all color channels are above the threshold
is_white = np.all(colors > white_threshold, axis=1)

# Create a new point cloud with only the white points
remaining_cloud = remaining_cloud.select_by_index(np.where(is_white)[0])

o3d.visualization.draw_geometries([remaining_cloud])


########################################################

""" Statistical outlier removal """
# nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
# std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud. 
# The lower this number the more aggressive the filter will be.

cl, ind = remaining_cloud.remove_statistical_outlier(nb_neighbors=10,
                                                    std_ratio=0.2)

# Remove stat outliers. ind is the indices of points which are inliers.
remaining_cloud = remaining_cloud.select_by_index(ind)


# Final visualization with black background
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(remaining_cloud)
opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])  # Set background color to black
vis.run()
vis.destroy_window()