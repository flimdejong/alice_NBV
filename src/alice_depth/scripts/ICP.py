import open3d as o3d
import numpy as np
import copy
import os

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

# Preprocess functions extracts the geometric features
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)

    # # Define the additional translation values
    # additional_translation = np.array([0, 0.0, 0])  # Replace x, y, z with the desired translation values

    # # Apply the additional translation to the already translated point cloud
    # source_down_rotated.translate(additional_translation)

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
    return result #Result is a transformation that can turn the source into target

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

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def filter_non_symmetric_points(pcd, radius, height, threshold):
    # Compute the centroid of the point cloud
    centroid = np.mean(np.asarray(pcd.points), axis=0)

    # Create a cylinder primitive
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    cylinder.translate(centroid)

    # Compute the distance from each point to the cylinder surface
    distances = np.asarray(pcd.compute_point_cloud_distance(cylinder.sample_points_uniformly(number_of_points=100000)))

    # Filter points based on the distance threshold
    mask = distances > threshold
    return pcd.select_by_index(np.where(mask)[0])

# Define the transformation matrix to align ROS and Open3D coordinate systems
ros_to_open3d_transform = np.array([[0, 0, 1, 0],
                                    [-1, 0, 0, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]])

base_dir = "/home/flimdejong/catkin_ws"
input_dir = os.path.join(base_dir, "stanford_bunny_processed")

source_file = os.path.join(input_dir, "stanford_bunny_low.pcd")
target_file = os.path.join(input_dir, "stanford_bunny_mid.pcd")

# Load the source and target point clouds from the .pcd files
source = o3d.io.read_point_cloud(source_file)
target = o3d.io.read_point_cloud(target_file)

# # Filter the non-symmetric part of the source mug
# radius = 0.04  # Adjust the radius based on your mug's dimensions
# height = 0.17  # Adjust the height based on your mug's dimensions
# threshold = 0.04  # Adjust the distance threshold as needed
# source_ransac = filter_non_symmetric_points(source, radius, height, threshold)

# # Filter the non-symmetric part of the target mug
# target_ransac = filter_non_symmetric_points(target, radius, height, threshold)

# o3d.visualization.draw_geometries([source_ransac])
# o3d.visualization.draw_geometries([target_ransac])

# source.rotate(source.get_rotation_matrix_from_xyz((0, np.pi / 3.8, 0)), center=(0, 0, 0))

# # Compute the centers of the source and target point clouds
# source_centroid = np.mean(np.asarray(source_ransac.points), axis=0)
# target_centroid = np.mean(np.asarray(target_ransac.points), axis=0)

# print(f'Center of source_center: {source_centroid}')
# print(f'Center of target_center: {target_centroid}')

# # Translate the source and target point clouds to align their centers
# source.translate(-source_centroid)
# target.translate(-target_centroid)

# print(f'Center of source_center after: {source.get_center()}')
# print(f'Center of target_center after: {target.get_center()}')

# #o3d.visualization.draw([source, target])

# Apply the transformation to align with Open3D coordinate system

#Set voxel size
voxel_size = 0.005

# This TF matrix is from Pose1 (x_low) to Pose2 (x_high)
transformation_matrix = [
 [ 9.99964044e-01, -4.38864689e-04,  8.46871074e-03, -4.32650074e-04],
 [ 6.98888227e-04,  9.99527615e-01, -3.07255412e-02, -9.34659714e-03],
 [-8.45122589e-03,  3.07303551e-02,  9.99491982e-01, -6.90924362e-02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]


# Create coordinate frame geometries
coord_frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
coord_frame_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25)

#Transform it all into Open3D coordinate plane
coord_frame_1.transform(ros_to_open3d_transform)
coord_frame_2.transform(ros_to_open3d_transform)
source.transform(ros_to_open3d_transform)
target.transform(ros_to_open3d_transform)


# Apply the transformations to the coordinate frames
coord_frame_2.transform(transformation_matrix)

# Apply the transform to target to make it align with source
target.transform(transformation_matrix)

# Visualize the transformed coordinate frames
o3d.visualization.draw_geometries([source, target, coord_frame_1, coord_frame_2])


#Preprocessing for RANSAC
source_ransac_down, source_ransac_fpfh = preprocess_point_cloud(source, voxel_size)
target_ransac_down, target_ransac_fpfh = preprocess_point_cloud(target, voxel_size)


#Global registration using RANSAC, returns global transformation matrix
result_ransac = execute_global_registration(source_ransac_down, target_ransac_down,
                                            source_ransac_fpfh, target_ransac_fpfh,
                                            voxel_size)
print(result_ransac)

# Apply the RANSAC transformation to the source point cloud
source.transform(result_ransac.transformation)

# Shows result after initial global registration
draw_registration_result(source_ransac_down, target_ransac_down, result_ransac.transformation)

#############################################

#Preprocessing for ICP
source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

# Refine the alignment using ICP on the full-sized point cloud
target.estimate_normals()
result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)
print(result_icp)

# Apply the ICP transformation to the source point cloud
source.transform(result_icp.transformation)

draw_registration_result(source, target, result_icp.transformation)


######## Merge pointclouds #######
##################################

merged_pcd = o3d.geometry.PointCloud()

# Concatenate the points from the target point cloud and the registered source point cloud
merged_pcd.points = o3d.utility.Vector3dVector(np.concatenate((target.points, source.points), axis=0))

# Update the normals of the merged point cloud (if required)
merged_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Visualize the merged point cloud
o3d.visualization.draw_geometries([merged_pcd])

# Save the merged point cloud to a file
output_filename = "processed_pcd/stanford_bunny_x_low_mid.pcd"
o3d.io.write_point_cloud(output_filename, merged_pcd)

# Create a visualizer object
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point clouds to the visualizer
vis.add_geometry(source)
vis.add_geometry(target)
#vis.add_geometry(merged_pcd)

# Set different colors for each point cloud
source.paint_uniform_color([1, 0, 0])  # Red
target.paint_uniform_color([0, 1, 0])  # Green
merged_pcd.paint_uniform_color([1, 1, 0])  # Yellow

# Run the visualization
vis.run()
vis.destroy_window()

##################################################


# # Estimate normal vectors for the target point cloud
# target.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
# )

# # Set the maximum correspondence distance (adjust as needed)
# distance_threshold = 0.02

# # Set the initial transformation matrix (identity matrix)
# initial_transform = np.identity(4)

# # Perform ICP point-to-plane registration
# result_icp = o3d.pipelines.registration.registration_icp(
#     source, target, distance_threshold, initial_transform,
#     o3d.pipelines.registration.TransformationEstimationPointToPlane()
# )

# # Get the final transformation matrix
# transformation_matrix = result_icp.transformation

# # Apply the transformation to the source pointcloud
# transformed_source = source.transform(transformation_matrix)

# # Visualize the aligned pointclouds
# o3d.visualization.draw_geometries([transformed_source, target])

# # Create a new pointcloud to store the combined result
# combined_pointcloud = o3d.geometry.PointCloud()


# # Combine the points from the transformed source and target pointclouds
# combined_pointcloud.points = o3d.utility.Vector3dVector(
#     np.concatenate((np.asarray(transformed_source.points), np.asarray(target.points)), axis=0)
# )

# # Combine the colors from the transformed source and target pointclouds (if applicable)
# if transformed_source.has_colors() and target.has_colors():
#     combined_pointcloud.colors = o3d.utility.Vector3dVector(
#         np.concatenate((np.asarray(transformed_source.colors), np.asarray(target.colors)), axis=0)
#     )

# # Save the combined pointcloud
# output_path = "processed_pcd/mok34.pcd"
# o3d.io.write_point_cloud(output_path, combined_pointcloud)