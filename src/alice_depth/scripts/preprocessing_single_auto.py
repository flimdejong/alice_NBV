import open3d as o3d

def preprocess_and_save(input_file, output_file):
    pcd = o3d.io.read_point_cloud(input_file)

    # 0.005 is too big. 0.003 is kinda fine.
    # Downsample pointcloud
    pcd = pcd.voxel_down_sample(voxel_size=0.003)

    # Applying plane segmentation using RANSAC (for background removal)
    # To find the plane with the largest support in the point cloud, we can use segment_plane. 
    # The method has three arguments: distance_threshold defines the maximum distance a point can have to an estimated plane to be considered an inlier
    # ransac_n defines the number of points that are randomly sampled to estimate a plane, and num_iterations
    # Applying plane segmentation using RANSAC (for floor removal)

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=2000)

    # Remove floor points from the point cloud
    removed_points = pcd.select_by_index(inliers)

    # The remaining PC
    remaining_cloud = pcd.select_by_index(inliers, invert = True)


    ########################################################

    plane_model, inliers = remaining_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=2000)

    # Remove floor points from the point cloud
    removed_points = remaining_cloud.select_by_index(inliers)

    # The remaining PC
    remaining_cloud = remaining_cloud.select_by_index(inliers, invert = True)


    ########################################

    plane_model, inliers = remaining_cloud.segment_plane(distance_threshold=0.001, ransac_n=3, num_iterations=500)

    # Remove floor points from the point cloud
    removed_points = remaining_cloud.select_by_index(inliers)

    # The remaining PC
    remaining_cloud = remaining_cloud.select_by_index(inliers, invert = True)

    # Statistical outlier removal
    # nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
    # std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud. 
    # The lower this number the more aggressive the filter will be.

    print("Statistical oulier removal")
    cl, ind = remaining_cloud.remove_statistical_outlier(nb_neighbors=30,
                                                        std_ratio=0.1)

    # Remove floor points from the point cloud, ind is the indices of points which are NOT outliers
    removed_points = remaining_cloud.select_by_index(ind, invert = True)

    # The remaining PC
    remaining_cloud = remaining_cloud.select_by_index(ind)

    # Save the processed point cloud
    o3d.io.write_point_cloud(output_file, remaining_cloud)
