#!/usr/bin/env python3

import open3d as o3d
import numpy as np

def preprocess_pc(input_file, output_file):

    # Load the pcd
    pcd = o3d.io.read_point_cloud(input_file)

    white_threshold = 0.67
    sample_percentage = 1

    # Get points and colors
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # Check if all color channels are above the threshold
    is_white = np.all(colors > white_threshold, axis=1)

    # Create a new point cloud with only the white points
    white_pcd = pcd.select_by_index(np.where(is_white)[0])
    white_colors = colors[np.where(is_white)[0]]
 

    # DBSCAN clustering
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            white_pcd.cluster_dbscan(eps=0.04, min_points=50, print_progress=True))


    # Get unique labels, excluding label -1 for noise
    unique_labels = np.unique(labels)
    unique_labels = unique_labels[unique_labels != -1]

    # Count number of points in each cluster
    label_counts = np.bincount(labels[labels != -1])

    # Calculate number of points to sample from each cluster
    sample_sizes = np.maximum(1, (label_counts * sample_percentage).astype(int))

    # Create a mask for sampled points
    sample_mask = np.zeros_like(labels, dtype=bool)

    # Sample points for each cluster
    for label, sample_size in zip(unique_labels, sample_sizes):
        cluster_indices = np.where(labels == label)[0]
        sampled_indices = np.random.choice(cluster_indices, sample_size, replace=False)
        sample_mask[sampled_indices] = True

    # Calculate total whiteness for sampled points in each cluster
    cluster_total_whiteness = np.array([
        np.sum(white_colors[(labels == label) & sample_mask].max(axis=1))
        for label in unique_labels
    ])

    # Find the best cluster
    best_cluster_index = np.argmax(cluster_total_whiteness)
    best_cluster_label = unique_labels[best_cluster_index]
    best_cluster_total_whiteness = cluster_total_whiteness[best_cluster_index]

    # Get sampled points of the best cluster
    best_cluster_mask = (labels == best_cluster_label) & sample_mask
    best_cluster_points = np.asarray(white_pcd.points)[best_cluster_mask]
    best_cluster_colors = white_colors[best_cluster_mask]

    # Create a new PointCloud object for the best cluster
    best_cluster_pcd = o3d.geometry.PointCloud()
    best_cluster_pcd.points = o3d.utility.Vector3dVector(best_cluster_points)
    best_cluster_colors = o3d.utility.Vector3dVector(best_cluster_colors)

    print(f"Total whiteness of sampled points: {best_cluster_total_whiteness:.4f}")
    print(f"Number of sampled points in the selected cluster: {len(best_cluster_points)}")


    """ Statistical outlier removal """
    # nb_neighbors, which specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
    # std_ratio, which allows setting the threshold level based on the standard deviation of the average distances across the point cloud. 
    # The lower this number the more aggressive the filter will be.

    cl, ind = best_cluster_pcd.remove_statistical_outlier(nb_neighbors=15,
                                                        std_ratio=0.7)

    # Remove stat outliers. ind is the indices of points which are inliers.
    best_cluster_pcd = best_cluster_pcd.select_by_index(ind)

    # Save the processed point cloud
    o3d.io.write_point_cloud(output_file, best_cluster_pcd)


