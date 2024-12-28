import numpy as np
import laspy
import open3d as o3d

def visualize_landing_spot(original_las_path, best_spot_normalized):
    """
    Visualize the best landing spot on original LAS data
    
    Args:
    original_las_path: Path to the original LAS file
    best_spot_normalized: Normalized coordinates of best landing spot [x, y, z]
    """
    # Load the original LAS file
    las = laspy.read(original_las_path)
    
    # Get original points
    points = np.vstack((las.X, las.Y, las.Z)).transpose()
    
    # Calculate normalization factors from original data
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    
    # Denormalize the best spot coordinates to original scale
    best_spot_original = best_spot_normalized * (max_vals - min_vals) + min_vals
    
    # Create colors array (default white for all points)
    colors = np.ones((len(points), 3)) * 0.7  # Light grey for regular points
    
    # Find points near the best landing spot (within a radius)
    # Convert to meters or appropriate unit based on your data
    radius = np.mean(max_vals - min_vals) * 0.01  # 1% of mean dimension
    
    # Calculate distances from all points to best spot
    distances = np.linalg.norm(points - best_spot_original, axis=1)
    
    # Color points near best spot (green)
    nearby_points = distances < radius
    colors[nearby_points] = [0, 1, 0]  # Green for landing spot
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Create coordinate frame for reference
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=np.mean(max_vals - min_vals) * 0.1,
        origin=best_spot_original
    )
    
    # Print information about the landing spot
    print("\nBest Landing Spot Information:")
    print(f"Original coordinates: {best_spot_original}")
    print(f"Number of points in landing zone: {np.sum(nearby_points)}")
    print(f"Visualization radius: {radius:.2f} meters")
    
    # Visualize
    o3d.visualization.draw_geometries(
        [pcd, coord_frame],
        window_name="Best Landing Spot Visualization",
        width=1024,
        height=768
    )

# Best landing spot coordinates from your output
spot = eval(input("Enter the best landing spot coordinates: "))
best_spot_normalized = np.array(spot)

# Use the function
visualize_landing_spot("Globhe_SampleData_LiDAR_PointCloud.las", best_spot_normalized)