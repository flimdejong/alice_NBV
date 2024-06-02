import open3d as o3d
import numpy as np
import os

# Set the input and output directories
input_dir = ""
output_dir = "processed_pcd"

# Apply depth thresholding
depth_threshold = 0.8  # Adjust the threshold based on your scene

#Set voxel size
voxel_size = 0.001

# Iterate over the point cloud files
for i in range(1, 46):
    # Load the point cloud from the .pcd file
    input_path = os.path.join(input_dir, f"mok{i}.pcd")
    pcd = o3d.io.read_point_cloud(input_path)
    
    # Print the point cloud details
    print(f"Processing bottle{i}.pcd")
    print(pcd)
    
    # Apply depth thresholding
    pcd_without_background = pcd.select_by_index(np.where(np.asarray(pcd.points)[:, 2] < depth_threshold)[0])
    
    # Downsizing
    downpcd = pcd_without_background.voxel_down_sample(voxel_size=voxel_size)
    
    # Applying plane segmentation using RANSAC
    plane_model, inliers = downpcd.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=1000)
    
    # Extract floor points
    floor_cloud = downpcd.select_by_index(inliers)
    
    # Remove floor points from the original point cloud
    remaining_cloud = downpcd.select_by_index(inliers, invert=True)
    
    # Statistical outlier removal
    cl, ind = remaining_cloud.remove_statistical_outlier(nb_neighbors=5, std_ratio=0.4)
    
    # Visualize the results (optional)
    # o3d.visualization.draw_geometries([cl])
    
    # Save the processed point cloud
    output_path = os.path.join(output_dir, f"mok{i}.pcd")
    o3d.io.write_point_cloud(output_path, cl)

print("Processing completed.")