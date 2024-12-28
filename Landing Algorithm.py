import numpy as np
import pandas as pd
import laspy
from sklearn.decomposition import PCA
import open3d as o3d
import timeit

start = timeit.default_timer()
# Load LiDAR data
las = laspy.read('Globhe_SampleData_LiDAR_PointCloud.las')
planar_data = pd.DataFrame({
    "X": las.X,
    "Y": las.Y,
    "Z": las.Z
})

def normalize_coordinates(data):
    min_vals = data.min()
    max_vals = data.max()
    normalized = (data - min_vals) / (max_vals - min_vals)
    return normalized, min_vals, max_vals

normalized_data, min_vals, max_vals = normalize_coordinates(planar_data[['X', 'Y', 'Z']])
planar_data[['X', 'Y', 'Z']] = normalized_data
print("Data normalized to [0, 1] range")
print("Original scale factors:\n", max_vals - min_vals)

# Create Open3D point cloud object for calculations
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(planar_data[['X', 'Y', 'Z']].values)

# Transformation matrices for transforming points between reference frames
T_bL = np.eye(4)  # Identity matrix for simplicity
T_nb_t = np.eye(4)  # Identity matrix for simplicity
print("Transformation Matrix and Identity Matrix formed....")

def transform_points(points, T_bL, T_nb_t):
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    points_body_frame = T_bL @ points_homogeneous.T
    points_nav_frame = T_nb_t @ points_body_frame
    return points_nav_frame[:3, :].T

# Transform points to navigation frame
# Voxel downsampling to reduce point density
voxel_size = 0.01  # Adjust this value to control point density
point_cloud_downsampled = point_cloud.voxel_down_sample(voxel_size=voxel_size)
points_nav_frame = np.asarray(point_cloud_downsampled.points)
print(f"Downsampled from {len(point_cloud.points)} to {len(point_cloud_downsampled.points)} points")
print("Transformation of points to Navigation Frame done....")

# Octree implementation
class OctreeNode:
    def __init__(self, points=None):
        self.points = points
        self.children = [None] * 8
        self.is_leaf = points is not None

class Octree:
    def __init__(self, max_depth):
        self.max_depth = max_depth
        self.root = None

    def convert_from_point_cloud(self, point_cloud, size_expand=0.1):
        min_bound = np.min(point_cloud, axis=0) - size_expand
        max_bound = np.max(point_cloud, axis=0) + size_expand
        self.root = self._build_octree(point_cloud, min_bound, max_bound, 0)

    def _build_octree(self, points, min_bound, max_bound, depth):
        if depth >= self.max_depth or len(points) <= 1:
            return OctreeNode(points)
        
        mid_point = (min_bound + max_bound) / 2
        children = [[] for _ in range(8)]
        
        for point in points:
            index = 0
            if point[0] > mid_point[0]: index |= 1
            if point[1] > mid_point[1]: index |= 2
            if point[2] > mid_point[2]: index |= 4
            children[index].append(point)
        
        node = OctreeNode()
        for i in range(8):
            if children[i]:
                new_min_bound = min_bound.copy()
                new_max_bound = max_bound.copy()
                if i & 1: new_min_bound[0] = mid_point[0]
                else: new_max_bound[0] = mid_point[0]
                if i & 2: new_min_bound[1] = mid_point[1]
                else: new_max_bound[1] = mid_point[1]
                if i & 4: new_min_bound[2] = mid_point[2]
                else: new_max_bound[2] = mid_point[2]
                node.children[i] = self._build_octree(children[i], new_min_bound, new_max_bound, depth + 1)
        
        return node

    def traverse(self, func):
        def _traverse_node(node):
            if node is None:
                return
            func(node)
            for child in node.children:
                _traverse_node(child)
        _traverse_node(self.root)

def find_neighborhood(octree, point, radius):
    if not hasattr(octree, 'root') or octree.root is None:
        raise ValueError("Octree has no root. Ensure it is properly initialized.")
    neighborhood_points = []
    point_np = np.asarray(point)

    def search_node(node, min_bound, max_bound):
        if node.is_leaf:
            for p in node.points:
                if np.linalg.norm(p - point_np) <= radius:
                    neighborhood_points.append(p)
            return

        mid_point = (min_bound + max_bound) / 2
        for i, child in enumerate(node.children):
            if child is not None:
                child_min_bound = min_bound.copy()
                child_max_bound = max_bound.copy()
                if i & 1: child_min_bound[0] = mid_point[0]
                else: child_max_bound[0] = mid_point[0]
                if i & 2: child_min_bound[1] = mid_point[1]
                else: child_max_bound[1] = mid_point[1]
                if i & 4: child_min_bound[2] = mid_point[2]
                else: child_max_bound[2] = mid_point[2]
                
                search_node(child, child_min_bound, child_max_bound)

    search_node(octree.root, np.min(points_nav_frame, axis=0), np.max(points_nav_frame, axis=0))
    return np.array(neighborhood_points)

# Octree creation
octree = Octree(max_depth=8)
octree.convert_from_point_cloud(points_nav_frame)
print("Converted Point Cloud Data to Octree....")

# Plane Detection using PCA
def detect_plane_pca(neighborhood):
    if neighborhood.shape[0] < 3:
        raise ValueError("Neighborhood must contain at least 3 points for PCA.")
    
    pca = PCA(n_components=3)
    pca.fit(neighborhood)
    normal_vector = pca.components_[-1]
    centroid = np.mean(neighborhood, axis=0)
    explained_variance_ratio = pca.explained_variance_ratio_
    planarity = 1 - explained_variance_ratio[-1]  # Higher value indicates more planar surface
    print(f"PCA explained variance ratio: {explained_variance_ratio}")
    print(f"Planarity score: {planarity:.4f}")

    a, b, c = normal_vector
    d = -np.dot(normal_vector, centroid)
    return a, b, c, d, normal_vector

# Slope calculation
def calculate_plane_slope(normal_vector):
    vertical_vector = np.array([0, 0, 1])
    cos_theta = np.dot(vertical_vector, normal_vector) / np.linalg.norm(normal_vector)
    theta = np.arccos(cos_theta)
    return np.degrees(theta)

# Plane Evaluation and Classification
def evaluate_and_classify_planes(planes, max_slope, max_std_dev, weights):
    classified_planes = []
    
    for plane in planes:
        a, b, c, d, normal_vector, neighborhood = plane
        
        slope_angle = calculate_plane_slope(normal_vector)
        z_values = neighborhood[:, 2]
        std_dev_z = np.std(z_values)
        
        print(f"Evaluating plane - Slope: {slope_angle:.2f}°, StdDev: {std_dev_z:.2f}")
        
        # More lenient criteria
        max_slope = 45  # Increased from original
        max_std_dev = max_std_dev * 2  # Doubled the tolerance

        if slope_angle <= max_slope and std_dev_z <= max_std_dev:
            centroid = np.mean(neighborhood, axis=0)
            g1 = 20 * (1 - slope_angle/max_slope)
            g2 = 20 * (1 - std_dev_z/max_std_dev)
            g3, g4 = 20, 20
            
            spot_grade = (g1 * weights[0] + g2 * weights[1] + g3 * weights[2] + g4 * weights[3]) / (20 * sum(weights))
            classified_planes.append((centroid, spot_grade))
            print(f"Plane classified with score: {spot_grade:.2f}")
        else:
            print("Plane rejected - exceeded slope or stddev criteria")
    
    classified_planes.sort(key=lambda x: x[1], reverse=True)
    return classified_planes

def update_scores(classified_planes, uav_position, weights):
    if not classified_planes:
        return []
        
    # Find max distance for normalization
    max_distance = max(np.linalg.norm(centroid - uav_position) for centroid, _ in classified_planes)
    
    updated_planes = []
    for centroid, suitability_score in classified_planes:
        # Normalize distance to [0,1] range
        distance = np.linalg.norm(centroid - uav_position)
        normalized_distance = distance / max_distance if max_distance > 0 else 0
        
        # Calculate operational score
        distance_score = 1 - normalized_distance  # Higher score for closer spots
        
        # Combine suitability and operational scores
        # Weighted average favoring suitability (0.7) over distance (0.3)
        final_score = (0.7 * suitability_score) + (0.3 * distance_score)
        updated_planes.append((centroid, final_score))
    
    return sorted(updated_planes, key=lambda x: x[1], reverse=True)

# Multi-point sampling for better coverage
num_samples = 100  # Number of points to sample
best_candidates = []

print("\nStarting landing spot analysis...")
print("Point cloud bounds:", points_nav_frame.min(axis=0), points_nav_frame.max(axis=0))

# Calculate appropriate radius based on data scale
data_range = points_nav_frame.max(axis=0) - points_nav_frame.min(axis=0)
radius = 0.05 * np.mean(data_range)  # Use 5% of average dimension
print(f"Analysis radius: {radius:.2f}")

# Sample multiple points across the point cloud
for i in range(num_samples):
    # Randomly sample points, preferring lower elevation points
    z_values = points_nav_frame[:, 2]
    z_weights = 1 / (1 + z_values - np.min(z_values))  # Higher weight for lower elevation
    z_weights /= np.sum(z_weights)
    
    sample_idx = np.random.choice(len(points_nav_frame), p=z_weights)
    search_point = points_nav_frame[sample_idx]
    
    print(f"\nAnalyzing candidate {i+1}/{num_samples}")
    print(f"Search point: {search_point}")
    
    # Find neighborhood
    neighborhood = find_neighborhood(octree, search_point, radius)
    num_points = len(neighborhood)
    print(f"Found {num_points} points in neighborhood")
    
    if num_points >= 10:  # Require minimum 10 points for reliable plane fitting
        # Perform local ground plane analysis
        try:
            a, b, c, d, normal_vector = detect_plane_pca(neighborhood)
            
            # Calculate local statistics
            local_z_std = np.std(neighborhood[:, 2])
            local_slope = calculate_plane_slope(normal_vector)
            
            print(f"Local analysis - Slope: {local_slope:.2f}°, Z-StdDev: {local_z_std:.4f}")
            
            # More stringent criteria for landing spots
            max_slope = 30.0  # Maximum 15 degrees slope
            max_local_std = 0.2 * np.mean(data_range)  # Scale-appropriate threshold
            
            if local_slope <= max_slope and local_z_std <= max_local_std:
                # Calculate quality metrics
                planarity_score = 1.0 - (local_slope / max_slope)
                flatness_score = 1.0 - (local_z_std / max_local_std)
                density_score = min(1.0, num_points / 1000)  # Normalize point density
                
                # Weighted scoring
                weights = {
                    'planarity': 0.4,
                    'flatness': 0.4,
                    'density': 0.2
                }
                
                total_score = (planarity_score * weights['planarity'] + 
                             flatness_score * weights['flatness'] + 
                             density_score * weights['density'])
                
                centroid = np.mean(neighborhood, axis=0)
                best_candidates.append({
                    'centroid': centroid,
                    'score': total_score,
                    'slope': local_slope,
                    'std_dev': local_z_std,
                    'num_points': num_points
                })
                
                print(f"Candidate accepted - Score: {total_score:.3f}")
            else:
                print("Candidate rejected - Exceeds slope or roughness criteria")
        except Exception as e:
            print(f"Error analyzing neighborhood: {e}")
    else:
        print("Insufficient points for analysis")

print("\nTime taken:", round(timeit.default_timer() - start, 2), "seconds")
# Process results
if best_candidates:
    # Sort by score
    best_candidates.sort(key=lambda x: x['score'], reverse=True)
    
    print("\nTop Landing Spots Found:")
    for i, spot in enumerate(best_candidates[:3]):  # Show top 3
        print(f"\nSpot {i+1}:")
        print(f"Score: {spot['score']:.3f}")
        print(f"Slope: {spot['slope']:.2f}°")
        print(f"Surface Roughness: {spot['std_dev']:.4f}")
        print(f"Supporting Points: {spot['num_points']}")
        print(f"Location: {spot['centroid']}")
    
    # Visualize the best spot
    best_spot = best_candidates[0]
    best_landing_spot = best_spot['centroid']
    
    # Create visualization
    colors = np.zeros((len(planar_data), 3))  # Initialize black
    
    # Mark the top 3 spots with different colors
    color_map = [[0, 1, 0], [0, 0.8, 0], [0, 0.6, 0]]  # Green gradient
    for i, spot in enumerate(best_candidates[:3]):
        closest_idx = np.argmin(np.linalg.norm(points_nav_frame - spot['centroid'], axis=1))
        colors[closest_idx] = color_map[i]
    
    # Save visualization
    output_las = laspy.LasData(las.header)
    output_las.X = las.X
    output_las.Y = las.Y
    output_las.Z = las.Z
    output_las.red = (colors[:, 0] * 65535).astype(int)
    output_las.green = (colors[:, 1] * 65535).astype(int)
    output_las.blue = (colors[:, 2] * 65535).astype(int)
    
    output_file = "BestLandingSpots.las"
    output_las.write(output_file)
    print(f"\nVisualization saved to {output_file}")
    def visualize_las_with_open3d(las_file):
            las = laspy.read(las_file)

            # Extract point cloud data
            points = np.vstack((las.X, las.Y, las.Z)).T
            colors = np.vstack((las.red, las.green, las.blue)).T / 65535  # Normalize RGB values

            # Create Open3D point cloud object
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # Visualize the point cloud
            o3d.visualization.draw_geometries([pcd], window_name="Best Landing Spot Visualization")
    # Visualize the results
    visualize_las_with_open3d(output_file)
else:
    print("\nNo suitable landing spots found.")

