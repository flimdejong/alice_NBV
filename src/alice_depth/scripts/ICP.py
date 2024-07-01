#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import copy
import os
import rospy
from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension

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

def transform_cb(msg):
    global relative_transform, new_transform_received

    # Reconstruct the matrix
    relative_transform = np.array(msg.data).reshape((4, 4), order='C')

    print("Matrix received:")
    print(relative_transform)

    # Set new_transform_received to True
    new_transform_received = True

def move_completed_cb(msg):
    global move_completed
    move_completed = msg.data
    #rospy.loginfo("Move completed: %s", move_completed)

def point_to_point_icp(source, target, threshold, trans_init):

    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    #draw_registration_result(source, target, reg_p2p.transformation)

    return reg_p2p.transformation

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, initial_transform):
    distance_threshold = voxel_size * 0.4
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


run = "2"

# Some global variables
# relative_transform = np.zeros((4, 4))
# new_transform_received = False
move_completed = False

# Intialize node
rospy.init_node('ICP', anonymous=True)

# subscriber that listens to the relative transforms
# transform_sub = rospy.Subscriber('/transformation_matrix', Float64MultiArray, transform_cb)
move_completed_sub = rospy.Subscriber('/move_completed', Bool, move_completed_cb)

# # Define the transformation matrix to align ROS and Open3D coordinate systems
# ros_to_open3d_transform = np.array([[0, 0, 1, 0],
#                                     [-1, 0, 0, 0],
#                                     [0, -1, 0, 0],
#                                     [0, 0, 0, 1]])

# wait until we get a move_completed True, so we begin our sequence.
while move_completed == False:
    rospy.sleep(0.1)

base_dir = "/home/flimdejong/catkin_ws/PC"
input_dir = os.path.join(base_dir, f"stanford_bunny_run_{run}_processed")
output_dir = os.path.join(base_dir, f"stanford_bunny_run_{run}_merged")

# Add try and except block to catch errors.

try:
    # For the first run, wait a little until the first pc has been published.

    rospy.logwarn("Wait a little for preprocessing to finish")
    rospy.sleep(3) 
    source_file = os.path.join(input_dir, f"stanford_bunny_run_{run}_1_processed.pcd")

    # Load the initial pointcloud as source
    source = o3d.io.read_point_cloud(source_file)

    # # Transform source into Open3D coordinate system
    # source.transform(ros_to_open3d_transform)

    rospy.loginfo("First PC succesfully loaded")

except FileNotFoundError:
    print(f"Error: '{source_file}' not found")

except PermissionError:
    print("Permission error")

# Start counter for merge at 1. This specific the ID of the merge
merged_count = 1

# Initialize the counter, start at 2 since we load the first one as 1
target_count = 2

# Set prefix for saving the merged PC's
prefix = "stanford_bunny_run_1"
suffix = "merged"

# Set voxel size for registration
voxel_size = 0.001

move_completed = False

while True:

    # wait until we get a move_completed True, so we begin our sequence.
    while move_completed == False:
        rospy.sleep(0.1)

    rospy.loginfo("Move_completed true, starting sequence")
    rospy.sleep(3)

    # Check if a new pointcloud file exists, starts at processed_2
    target_file = os.path.join(input_dir, f"stanford_bunny_run_{run}_{target_count}_processed.pcd")
    if os.path.exists(target_file):

        rospy.loginfo ("Found the 2 PC's, starting merging")

        # Load the new pointcloud
        target = o3d.io.read_point_cloud(target_file)

        # # Convert target pc to Open3D coordinate frame
        # target.transform(ros_to_open3d_transform)

        # # Apply the transform to target to make it align with source
        # target.transform(relative_transform)

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
        result_ransac = execute_fast_global_registration(source_ransac_down, target_ransac_down,
                                                    source_ransac_fpfh, target_ransac_fpfh,
                                                    voxel_size)
        print(result_ransac)

        # Apply the RANSAC transformation to the target point cloud for furthur refinement
        draw_registration_result(source, target, result_ransac.transformation)

        ######################
        ##### Apply ICP ######
        ######################
        """ 
        Apply ICP (point-to-point) for furthur refinement
        """

        icp_transformation = point_to_point_icp(source, target, 0.02, result_ransac.transformation)

        draw_registration_result(source, target, icp_transformation)

        source.transform(icp_transformation)

        # #Preprocessing for ICP
        # source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        # target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

        # # Refine the alignment using ICP on the full-target_downsized point cloud

        # result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
        #                                 voxel_size, result_ransac.transformation)
        # print(result_icp)
        # draw_registration_result(source_down, target_down, result_icp.transformation)

        # # Apply the ICP transformation to the source point cloud
        # source.transform(result_icp.transformation)


        """ Combining pointclouds """

        # Combine the pointclouds
        pcd_combined = source + target

        # Downsample the result to 0.001 to keep pc sparse
        pcd_combined = pcd_combined.voxel_down_sample(voxel_size)

        # Generate the output filename based on the number of merged pointclouds
        output_filename = f"{prefix}_{merged_count}_{suffix}.pcd"
        output_file_path = os.path.join(output_dir, output_filename)
        o3d.io.write_point_cloud(output_file_path, pcd_combined)

        print(f"Merged pointcloud saved as {output_filename}")

        # Increment the counter for the number of merged pointclouds + the target
        merged_count += 1
        target_count += 1

        # Source is the combined PC (make it equal to the result of the n-th reconstruction)
        source = pcd_combined

    else:
        rospy.sleep(0.1)
        rospy.logerr("next pcd not found")

    # Set to false and wait for another move
    move_completed = False