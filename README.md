 # IRoc Landing Algorithm

An autonomous landing spot detection algorithm for UAVs using LiDAR point cloud data. This algorithm analyzes terrain characteristics to identify optimal landing locations based on factors like slope, surface roughness, and point density.

## Dataset

The algorithm uses LiDAR point cloud data from Globhe. You can download the sample dataset here:  
[Globhe Sample LiDAR Point Cloud Dataset](https://share.hsforms.com/1pN-7jq6HTby9FXlgqEz3NAbnbxm)

## Project Structure

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

## Usage

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

## Output

The algorithm will:
- Generate a list of potential landing spots with quality scores
- Save a visualization file (`BestLandingSpots.las`)
- Display an interactive 3D visualization
- Provide detailed metrics for each candidate landing spot

## Requirements

- Python 3.10 or higher
- See `requirements.txt` for package dependencies
