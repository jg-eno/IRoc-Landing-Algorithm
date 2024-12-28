# IRoc Landing Algorithm

An autonomous landing spot detection algorithm for UAVs using LiDAR point cloud data. This algorithm analyzes terrain characteristics to identify optimal landing locations based on factors like slope, surface roughness, and point density.

---

## **Algorithm Overview**

1. **Data Loading and Normalization**:
   - The LiDAR data is loaded from a `.las` file and normalized to a [0, 1] range for easier processing.
   - The normalized data is converted into an Open3D point cloud object for further analysis.

2. **Point Cloud Downsampling**:
   - The point cloud is downsampled using voxel downsampling to reduce the number of points while preserving the overall structure.

3. **Octree Construction**:
   - An octree data structure is built from the downsampled point cloud to efficiently query neighborhoods around specific points.

4. **Plane Detection**:
   - For each sampled point, a neighborhood is queried using the octree.
   - Principal Component Analysis (PCA) is used to detect planar surfaces within the neighborhood. The normal vector of the plane and its planarity score are computed.

5. **Plane Evaluation**:
   - Each detected plane is evaluated based on its slope and surface roughness (standard deviation of Z-values).
   - Planes that meet the criteria (e.g., slope ≤ 30°, roughness ≤ threshold) are considered as potential landing spots.

6. **Scoring Mechanism**:
   - Each candidate landing spot is scored based on three key metrics:
     - **Planarity**: Measures how flat the surface is (based on the slope angle).
     - **Flatness**: Measures the surface roughness (based on the standard deviation of Z-values).
     - **Density**: Measures the number of supporting points in the neighborhood.
   - These metrics are combined into a weighted score, with higher weights given to planarity and flatness.

7. **Ranking and Visualization**:
   - The candidate spots are ranked based on their scores.
   - The top spots are visualized in the point cloud, with different colors indicating their ranking.

---

## **Scoring Mechanism**

The scoring mechanism evaluates each candidate landing spot based on three key metrics, which are combined into a weighted score:

1. **Planarity Score**:
   - Measures how flat the surface is.
   - Calculated as:  
     $$
     \text{Planarity Score} = 1.0 - \left(\frac{\text{Local Slope}}{\text{Max Slope}}\right)
     $$
   - A higher score indicates a flatter surface.

2. **Flatness Score**:
   - Measures the surface roughness (standard deviation of Z-values).
   - Calculated as:
     $$
     \text{Flatness Score} = 1.0 - \left(\frac{\text{Local StdDev}}{\text{Max StdDev}}\right)
     $$
   - A higher score indicates a smoother surface.

3. **Density Score**:
   - Measures the number of supporting points in the neighborhood.
   - Calculated as:  
     $$
   \text{Density Score} = \min\left(1.0, \frac{\text{Number of Points}}{1000}\right)
   $$
   - A higher score indicates a denser neighborhood, which provides more reliable plane fitting.

4. **Weighted Total Score**:
   - The total score is a weighted combination of the three metrics:  
     $$
     \text{Total Score} = (\text{Planarity Score} \times 0.4) + (\text{Flatness Score} \times 0.4) + (\text{Density Score} \times 0.2)
     $$
   - The weights emphasize planarity and flatness over density.

---

## **Dataset**

The algorithm uses LiDAR point cloud data from Globhe. You can download the sample dataset here:  
[Globhe Sample LiDAR Point Cloud Dataset](https://share.hsforms.com/1pN-7jq6HTby9FXlgqEz3NAbnbxm)

---

## **Project Structure**

The project consists of three main files:

### 1. `requirements.txt`
Contains all necessary Python dependencies:
```
laspy==2.0.0
numpy==1.26.4
open3d==0.18.0
pandas==2.2.3
scikit-learn==1.6.0
```

### 2. `Landing Algorithm.py`
The core algorithm that:
- Loads and processes LiDAR point cloud data
- Implements Octree-based spatial indexing for efficient neighbor searching
- Uses PCA for plane detection and analysis
- Evaluates potential landing spots based on:
  - Slope analysis
  - Surface roughness
  - Point density
  - Local terrain characteristics
- Outputs and visualizes the top landing spots

### 3. `Visual.py`
Visualization utility that:
- Renders the original point cloud data
- Highlights the selected landing spot
- Provides an interactive 3D visualization using Open3D
- Displays coordinate frames and landing zone markers

---

## **Usage**

1. Install the required dependencies:  

   ### Using Conda:
   ```bash
   conda create -n iroc_env -c conda-forge laspy=2.0.0 numpy=1.26.4 open3d=0.18.0 pandas=2.2.3 scikit-learn=1.6.0
   conda activate iroc_env
   ```

   ### Using pip:
   ```bash
   pip install -r requirements.txt
   ```

2. Download the sample LiDAR dataset from the provided link.

3. Run the landing spot detection:
   ```bash
   python "Landing Algorithm.py"
   ```

4. Once the algorithm identifies the best landing spot, visualize it:
   ```bash
   python Visual.py
   ```
   When prompted, enter the landing spot coordinates output by the algorithm.

---

## **Output**

The algorithm will:
- Generate a list of potential landing spots with quality scores
- Save a visualization file (`BestLandingSpots.las`)
- Display an interactive 3D visualization
- Provide detailed metrics for each candidate landing spot

---

## **Key Parameters**

- **Max Slope**: The maximum allowable slope for a landing spot (e.g., 30°).
- **Max StdDev**: The maximum allowable surface roughness (e.g., 20% of the data range).
- **Weights**: The weights assigned to planarity (0.4), flatness (0.4), and density (0.2).

---

## **Strengths**

- **Efficiency**: The use of octrees and downsampling ensures efficient processing of large point clouds.
- **Robustness**: The scoring mechanism considers multiple factors (slope, roughness, density) to ensure reliable landing spot detection.
- **Visualization**: The results are visually intuitive, making it easy to identify the best landing spots.

---

## **Limitations**

- **Parameter Sensitivity**: The results depend on the choice of parameters (e.g., max slope, max stddev, weights).
- **Data Quality**: The algorithm assumes that the input LiDAR data is accurate and sufficiently dense.

---

## **Applications**

This algorithm is particularly useful for:
- Autonomous UAV navigation and landing.
- Terrain analysis for robotics and autonomous vehicles.
- Site selection for infrastructure development.

By combining efficient data processing with a robust scoring mechanism, the algorithm provides a reliable solution for identifying suitable landing spots in complex environments.

---

## **Requirements**

- Python 3.10 or higher
- See `requirements.txt` for package dependencies
