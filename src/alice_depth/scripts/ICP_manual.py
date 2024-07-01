#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import copy
import os
import rospy
from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
import matplotlib as plt
from skimage import color
import colorsys

"""
ICP.py is a py script that takes two pointclouds, preprocesses them and finally merges them by using their relative transforms, global registration on a 
downsampled voxel size and ICP for the final alignment.
"""

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

# Preprocess functions extracts the geometric features
def preprocess_point_cloud(pcd, voxel_size):
    #print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    #print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,#
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, initial_transform):
    distance_threshold = voxel_size * 0.4
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def refine_registration_color(source, target):

    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)

    print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
        current_transformation = result_icp.transformation
        print(result_icp)

        return current_transformation

def visualise(source, target):
    
    # Set the color of the target point cloud to red
    #target.paint_uniform_color([1, 0, 0])  # RGB for red

    # Final visualization with black background
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add geometries separately
    vis.add_geometry(target)
    vis.add_geometry(source)

    # Set render options
    opt = vis.get_render_option()
    #opt.background_color = np.asarray([0, 0, 0])  # Set background color to black

    # Update geometry to ensure changes take effect
    vis.update_geometry(target)
    vis.update_geometry(source)

    # Update rendering
    vis.poll_events()
    vis.update_renderer()

    # Run the visualizer
    vis.run()
    vis.destroy_window()

def point_to_point_icp(source, target, threshold, trans_init):

    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    draw_registration_result(source, target, reg_p2p.transformation)



base_dir = "/home/flimdejong/catkin_ws/PC"
input_dir = os.path.join(base_dir, "stanford_bunny_run_1_processed")

# Define the transformation matrix to align ROS and Open3D coordinate systems
ros_to_open3d_transform = np.array([[0, 0, 1, 0],
                                    [-1, 0, 0, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]])

relative_transform = np.array([
    [ 9.75399389e-01, -1.77490288e-03, -2.20437931e-01, -4.23558258e-03],
    [ 1.54688908e-03,  9.99998075e-01, -1.20698283e-03, -7.07445135e-02],
    [ 2.20439649e-01,  8.36297288e-04,  9.75400257e-01, -2.06162970e-01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
])

# Source
source_file = os.path.join(input_dir, "stanford_bunny_run_1_1_processed.pcd")
# Load the initial pointcloud as source
source = o3d.io.read_point_cloud(source_file)

# Target PC
target_file = os.path.join(input_dir, "stanford_bunny_run_1_2_processed.pcd")
# Load the new pointcloud
target = o3d.io.read_point_cloud(target_file)

#relative_transform = np.linalg.inv(relative_transform)

# Now transform both to Open3D coordinate system
source.transform(ros_to_open3d_transform)
target.transform(ros_to_open3d_transform)

# Apply the relative transform to target (in ROS coordinate system)
#target.transform(relative_transform)

voxel_size = 0.001

# visualise(source, target)


# Change colors to differentiate point clouds
# source.paint_uniform_color([1, 0, 0])  # Red
# target.paint_uniform_color([0, 1, 0])  # Green

# Visualize
# o3d.visualization.draw_geometries([source, target, 
#                                    source_frame, target_frame],
#                                   window_name="Transform Test Visualization",
#                                   width=1024, height=768,
#                                   left=50, top=50,
#                                   point_show_normal=False,
#                                   mesh_show_wireframe=True,
#                                   mesh_show_back_face=True)

# Now we have source and target. Source is the one we get first, we will transform target onto source

######################################
##### Apply Global Registration ######
######################################
""" 
Apply global registration for rough alignment
"""

#Preprocessing for RANSAC
source_ransac_down, source_ransac_fpfh = preprocess_point_cloud(source, voxel_size)
target_ransac_down, target_ransac_fpfh = preprocess_point_cloud(target, voxel_size)

#Global registration using RANSAC, returns global transformation matrix
result_ransac = execute_global_registration(source_ransac_down, target_ransac_down,
                                            source_ransac_fpfh, target_ransac_fpfh,
                                            voxel_size)
print(result_ransac)

#target.transform(result_ransac.transformation)

draw_registration_result(source, target, result_ransac.transformation)


################
# Testing
##########

# source_down = source.voxel_down_sample(voxel_size=0.03)
# target_down = target.voxel_down_sample(voxel_size=0.03)


""" Segmentation based on colors """

# # Get colors as a numpy array
# colors = np.asarray(source.colors)

# # Define color threshold for white
# white_threshold = 0.7

# # Check if all color channels are above the threshold
# is_white = np.all(colors > white_threshold, axis=1)

# # Create a new point cloud with only the white points
# source = source.select_by_index(np.where(is_white)[0])

# o3d.visualization.draw_geometries([source])



# # Get colors as a numpy array
# colors = np.asarray(target.colors)

# # Define color threshold for white # 0.4 is not a big improvement, if any over 0.5
# white_threshold = 0.62

# # Check if all color channels are above the threshold
# is_white = np.all(colors > white_threshold, axis=1)

# # # Create a new point cloud with only the white points
# # target = target.select_by_index(np.where(is_white)[0])


# # Create a new color array
# new_colors = np.copy(colors)

# # Set non-white points to red
# new_colors[~is_white] = [1, 0, 0]  # Red in RGB

# # Update the colors of the point cloud
# target.colors = o3d.utility.Vector3dVector(new_colors)

# # Visualize the result
# o3d.visualization.draw_geometries([target])


""" Statistical outlier removal """
# nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
# std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud. 
# The lower this number the more aggressive the filter will be.

# cl, ind = source.remove_statistical_outlier(nb_neighbors=15,
#                                                     std_ratio=2)

# # Remove stat outliers. ind is the indices of points which are inliers.
# source = source.select_by_index(ind)

# cl, ind = target.remove_statistical_outlier(nb_neighbors=5,
#                                                     std_ratio=0.8)

# # Remove stat outliers. ind is the indices of points which are inliers.
# remaining_cloud = target.select_by_index(ind)

# # Remove stat outliers. ind is the indices of points which are inliers.
# target = target.select_by_index(ind)


######################
##### Apply ICP ######
######################
""" 
Apply ICP for furthur refinement
"""

#Preprocessing for ICP
source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

# Refine the alignment using ICP on the full-target_downsized point cloud

result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                 voxel_size, result_ransac.transformation)
print(result_icp)
draw_registration_result(source_down, target_down, result_icp.transformation)

# Apply the ICP transformation to the source point cloud
source.transform(result_icp.transformation)

# # Combine the pointclouds
# pcd_combined = source + target




# # Apply colored ICP

# icp_colored_transform = refine_registration_color(source, target)

# # Apply the ICP transformation to the source point cloud
# target.transform(icp_colored_transform)

# visualise(target, source)


## Point to point ICP
#point_to_point_icp(source, target, 0.02, np.identity(4))

# def preprocessing(pcd, white_threshold, sample_percentage):

#     # # Downsample for performance reasons
#     # pcd_down = pcd.voxel_down_sample(voxel_size=0.001)

#     # Get points and colors
#     points = np.asarray(pcd.points)
#     colors = np.asarray(pcd.colors)

#     # Check if all color channels are above the threshold
#     is_white = np.all(colors > white_threshold, axis=1)

#     # Create a new point cloud with only the white points
#     white_pcd = pcd.select_by_index(np.where(is_white)[0])
#     white_colors = colors[np.where(is_white)[0]]


#     # DBSCAN clustering
#     with o3d.utility.VerbosityContextManager(
#             o3d.utility.VerbosityLevel.Debug) as cm:
#         labels = np.array(
#             white_pcd.cluster_dbscan(eps=0.04, min_points=50, print_progress=True))


#     # Get unique labels, excluding label -1 for noise
#     unique_labels = np.unique(labels)
#     unique_labels = unique_labels[unique_labels != -1]

#     # Count number of points in each cluster
#     label_counts = np.bincount(labels[labels != -1])

#     # Calculate number of points to sample from each cluster
#     sample_sizes = np.maximum(1, (label_counts * sample_percentage).astype(int))

#     # Create a mask for sampled points
#     sample_mask = np.zeros_like(labels, dtype=bool)

#     # Sample points for each cluster
#     for label, sample_size in zip(unique_labels, sample_sizes):
#         cluster_indices = np.where(labels == label)[0]
#         sampled_indices = np.random.choice(cluster_indices, sample_size, replace=False)
#         sample_mask[sampled_indices] = True

#     # Calculate mean white values for sampled points in each cluster
#     cluster_means = np.array([
#         np.mean(white_colors[(labels == label) & sample_mask].max(axis=1))
#         for label in unique_labels
#     ])

#     # Find the best cluster
#     best_cluster_index = np.argmax(cluster_means)
#     best_cluster_label = unique_labels[best_cluster_index]
#     best_cluster_mean = cluster_means[best_cluster_index]

#     # Get points of the best cluster
#     best_cluster_mask = (labels == best_cluster_label)
#     best_cluster_points = np.asarray(white_pcd.points)[best_cluster_mask]

#     # Create a new PointCloud object for the best cluster
#     best_cluster_pcd = o3d.geometry.PointCloud()
#     best_cluster_pcd.points = o3d.utility.Vector3dVector(best_cluster_points)

#     print(f"Mean white value: {best_cluster_mean:.4f}")
    
#     return best_cluster_pcd


# base_dir = "/home/flimdejong/catkin_ws/PC"

# source_file = os.path.join(base_dir, "hsv.pcd")

# # Load your point cloud
# pcd = o3d.io.read_point_cloud(source_file)

# # Filter out the background
# filtered_pcd = preprocessing(pcd, 0.7, 1)






# """ Statistical outlier removal """
# # nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
# # std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud. 
# # The lower this number the more aggressive the filter will be.

# cl, ind = filtered_pcd.remove_statistical_outlier(nb_neighbors=15,
#                                                     std_ratio=1)

# # Remove stat outliers. ind is the indices of points which are inliers.
# filtered_pcd = filtered_pcd.select_by_index(ind)

# # Final visualization with black background
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(filtered_pcd)
# opt = vis.get_render_option()
# opt.background_color = np.asarray([0, 0, 0])  # Set background color to black
# vis.run()
# vis.destroy_window()



# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     labels = np.array(
#         target.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

# max_label = labels.max()
# print(f"point cloud has {max_label + 1} clusters")
# colors = plt.colormaps['tab20'](labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# target.colors = o3d.utility.Vector3dVector(colors[:, :3])

# o3d.visualization.draw_geometries([target])

#################################################
# Input is pcd, white_threshold, sample_percentage
# bunny_pcd_source = preprocessing(source, 0.6, 0.5)
# bunny_pcd_target = preprocessing(target, 0.6, 0.5)

# print("Before ransac")
# o3d.visualization.draw_geometries([bunny_pcd_source, bunny_pcd_target])


# #Preprocessing for RANSAC
# source_ransac_down, source_ransac_fpfh = preprocess_point_cloud(source, voxel_size)
# target_ransac_down, target_ransac_fpfh = preprocess_point_cloud(target, voxel_size)

# #Global registration using RANSAC, returns global transformation matrix
# result_ransac = execute_global_registration(source_ransac_down, target_ransac_down,
#                                             source_ransac_fpfh, target_ransac_fpfh,
#                                             voxel_size)
# print(result_ransac)


# draw_registration_result(source, target, result_ransac.transformation)


# # # Point to point ICP
# point_to_point_icp(bunny_pcd_source, bunny_pcd_target, 0.02, result_ransac.transformation)




# merged_1 = source + target

# target_file = os.path.join(input_dir, "stanford_bunny_run_9_3.pcd")
# target = o3d.io.read_point_cloud(target_file)

# bunny_pcd_target = preprocessing(target)

# #Preprocessing for RANSAC
# source_ransac_down, source_ransac_fpfh = preprocess_point_cloud(merged_1, voxel_size)
# target_ransac_down, target_ransac_fpfh = preprocess_point_cloud(bunny_pcd_target, voxel_size)

# #Global registration using RANSAC, returns global transformation matrix
# result_ransac = execute_fast_global_registration(source_ransac_down, target_ransac_down,
#                                             source_ransac_fpfh, target_ransac_fpfh,
#                                             voxel_size)
# print(result_ransac)

# # Point to point ICP
# point_to_point_icp(merged_1, bunny_pcd_target, 0.02, result_ransac.transformation)

# merged_2 = merged_1 + target
# o3d.visualization.draw_geometries([merged_2]) # This is 3 PC

