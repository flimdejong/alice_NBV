import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
from mpl_toolkits.mplot3d import Axes3D

def visualize_hsv_distribution(pcd):
    colors = np.asarray(pcd.colors)
    colors_uint8 = (colors * 255).astype(np.uint8)
    hsv_colors = cv2.cvtColor(colors_uint8.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)

    fig = plt.figure(figsize=(20, 5))
    
    # 2D histograms
    ax1 = fig.add_subplot(141)
    ax1.set_title('Hue Distribution')
    ax1.hist(hsv_colors[:, 0], bins=180, range=(0, 180))
    ax1.set_xlabel('Hue')
    
    ax2 = fig.add_subplot(142)
    ax2.set_title('Saturation Distribution')
    ax2.hist(hsv_colors[:, 1], bins=256, range=(0, 256))
    ax2.set_xlabel('Saturation')
    
    ax3 = fig.add_subplot(143)
    ax3.set_title('Value Distribution')
    ax3.hist(hsv_colors[:, 2], bins=256, range=(0, 256))
    ax3.set_xlabel('Value')
    
    # 3D scatter plot
    ax4 = fig.add_subplot(144, projection='3d')
    ax4.set_title('3D HSV Distribution')
    sample_size = min(10000, hsv_colors.shape[0])  # Limit to 10000 points for performance
    sample_indices = np.random.choice(hsv_colors.shape[0], sample_size, replace=False)
    
    scatter = ax4.scatter(
        hsv_colors[sample_indices, 0], 
        hsv_colors[sample_indices, 1], 
        hsv_colors[sample_indices, 2], 
        c=colors[sample_indices], 
        marker='.'
    )
    ax4.set_xlabel('Hue')
    ax4.set_ylabel('Saturation')
    ax4.set_zlabel('Value')
    
    plt.tight_layout()
    plt.show()

    return hsv_colors

def analyze_hsv_distribution(pcd):
    colors = np.asarray(pcd.colors)
    colors_uint8 = (colors * 255).astype(np.uint8)
    colors_hsv = cv2.cvtColor(colors_uint8.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)

    print("HSV Distribution Statistics:")
    print(f"Min HSV: {colors_hsv.min(axis=0)}")
    print(f"Max HSV: {colors_hsv.max(axis=0)}")
    print(f"Mean HSV: {colors_hsv.mean(axis=0)}")
    print(f"Median HSV: {np.median(colors_hsv, axis=0)}")
    
    # Calculate the most common HSV values
    unique_hsv, counts = np.unique(colors_hsv, axis=0, return_counts=True)
    sorted_indices = np.argsort(counts)[::-1]
    
    print("\nMost common HSV values:")
    for i in range(min(5, len(unique_hsv))):
        hsv = unique_hsv[sorted_indices[i]]
        count = counts[sorted_indices[i]]
        percentage = (count / len(colors_hsv)) * 100
        print(f"HSV {hsv}: {percentage:.2f}% ({count} points)")

def filter_background(pcd, saturation_threshold=10, value_threshold=254):
    # Extract colors from the point cloud
    colors = np.asarray(pcd.colors)
    
    # Convert colors to uint8 and then to HSV
    colors_uint8 = (colors * 255).astype(np.uint8)
    hsv_colors = cv2.cvtColor(colors_uint8.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    
    # Keep points with saturation below 30 AND value above 250
    mask = (hsv_colors[:, 1] < saturation_threshold) & (hsv_colors[:, 2] > value_threshold)
    
    # Apply mask to points and colors
    filtered_points = np.asarray(pcd.points)[mask]
    filtered_colors = colors[mask]
    
    # Create new point cloud with filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_colors = o3d.utility.Vector3dVector(filtered_colors)
    
    return filtered_pcd



base_dir = "/home/flimdejong/catkin_ws/PC"

source_file = os.path.join(base_dir, "hsv.pcd")

# Load your point cloud
pcd = o3d.io.read_point_cloud(source_file)


# Filter out the background
filtered_pcd = filter_background(pcd)



# Final visualization with black background
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(filtered_pcd)
opt = vis.get_render_option()
opt.background_color = np.asarray([0, 0, 0])  # Set background color to black
vis.run()
vis.destroy_window()


# # Visualize or save the result
# o3d.visualization.draw_geometries([filtered_pcd])
o3d.io.write_point_cloud("filtered_point_cloud.pcd", filtered_pcd)



# # Check if the point cloud has colors
# if not pcd.has_colors():
#     print("Error: The point cloud does not have color information.")
# else:
#     # Visualize the HSV spectrum
#     visualize_hsv_distribution(pcd)
    
#     # Analyze HSV distribution
#     analyze_hsv_distribution(pcd)