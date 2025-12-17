Here is the complete updated `README.md`. I have updated the `cloud_fusion_node` section to reflect the high-performance "Ultra Fast" implementation (OpenMP, raw buffer processing, removal of filters) and updated the relevant usage examples. The details for the other nodes remain unchanged.


# multi_cloud_fusion

A ROS 2 package for fusing multiple point clouds from different sensors and projecting the result into top-view RGB and depth images.

## Overview

This package provides a two-stage processing pipeline:

1. **CloudFusionNode** - Synchronizes and fuses two point clouds into a single cloud in a target frame using a high-performance, parallelized pipeline.
2. **TopViewProjectorNode** - Projects the fused 3D point cloud into 2D top-view RGB and depth images.

Additionally, utility nodes are provided for:
- **icp_tf_node** - ICP-based transform estimation between point clouds
- **frame_tf_saver_node** - Saving TF transforms to YAML files

## Dependencies

- ROS 2 (tested with Humble/Jazzy)
- PCL (Point Cloud Library)
- OpenMP (for parallel processing)
- sensor_msgs
- tf2 and tf2_sensor_msgs
- message_filters

## Building

```bash
# From your workspace root
# Ensure you build in Release mode for maximum performance with OpenMP
colcon build --packages-select multi_cloud_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
````

## Nodes

### 1\. cloud\_fusion\_node

A high-performance node that fuses two time-synchronized point clouds into a common target frame.

**Key Features:**

  - **Ultra-Fast Processing:** Uses **OpenMP** to parallelize transformations across all CPU cores.
  - **Raw Buffer Manipulation:** Bypasses PCL conversion overhead by operating directly on ROS message memory buffers.
  - **Zero Filtering:** To minimize latency (achieving \~30Hz on Jetson AGX/Orin), all voxel grid, cropping, and outlier removal filters have been removed. This node performs strictly transform-and-merge operations.

**Subscribed Topics:**

  - `cloud1` (sensor\_msgs/PointCloud2) - First point cloud input
  - `cloud2` (sensor\_msgs/PointCloud2) - Second point cloud input

**Published Topics:**

  - `fused_cloud` (sensor\_msgs/PointCloud2) - Merged point cloud in target frame

**Parameters:**

  - `target_frame` (string, default: "morpho\_conveyor\_base") - The target coordinate frame to which both clouds will be transformed.

### 2\. top\_view\_projector\_node

Projects a 3D point cloud into 2D top-view RGB and depth images.

**Subscribed Topics:**

  - `cloud` (sensor\_msgs/PointCloud2) - Input point cloud

**Published Topics:**

  - `top_view/rgb` (sensor\_msgs/Image) - RGB image (encoding: "rgb8")
  - `top_view/depth` (sensor\_msgs/Image) - Depth image (encoding: "32FC1")

**Parameters:**

  - `image_width` (int, default: 640) - Output image width in pixels
  - `image_height` (int, default: 480) - Output image height in pixels
  - `x_min`, `x_max` (double, default: -1.0, 1.0) - ROI bounds in X axis (meters)
  - `y_min`, `y_max` (double, default: -1.0, 1.0) - ROI bounds in Y axis (meters)
  - `use_max_height` (bool, default: true) - Take highest z per pixel (else lowest)
  - `bg_r`, `bg_g`, `bg_b` (int, default: 0) - Background color (0-255)
  - `use_hole_filling` (bool, default: false) - Enable 4-neighbor hole filling
  - `hole_filling_iterations` (int, default: 1) - Number of filling iterations
  - `use_smoothing` (bool, default: false) - Enable box filter smoothing
  - `smoothing_kernel_size` (int, default: 3) - Kernel size (must be odd, \>=3)

### 3\. icp\_tf\_node

Estimates and publishes the transform between two point clouds using Iterative Closest Point (ICP) algorithm. Useful for initial calibration or dynamic transform estimation.

**Subscribed Topics:**

  - `source_topic` (sensor\_msgs/PointCloud2) - Source point cloud
  - `target_topic` (sensor\_msgs/PointCloud2) - Target point cloud

**Published Topics:**

  - Publishes TF transform: `target_frame` → `source_frame` (when `publish_tf` is true)

**Parameters:**

  - `source_topic` (string, default: "/camera/right/points") - Source point cloud topic
  - `target_topic` (string, default: "/camera/left/points") - Target point cloud topic
  - `icp_rate_hz` (double, default: 5.0) - Maximum ICP update rate in Hz
  - `voxel_leaf_size` (double, default: 0.01) - Voxel grid downsampling size; ≤0 disables
  - `icp_max_iterations` (int, default: 50) - Maximum ICP iterations
  - `icp_max_correspondence_distance` (double, default: 0.05) - Max point correspondence distance
  - `icp_transformation_epsilon` (double, default: 1e-8) - Transformation convergence threshold
  - `icp_euclidean_fitness_epsilon` (double, default: 1e-6) - Fitness convergence threshold
  - `fitness_score_threshold` (double, default: 0.5) - Reject ICP results above this fitness score
  - `use_previous_transform_as_initial_guess` (bool, default: true) - Use previous ICP result as initial guess
  - `publish_tf` (bool, default: true) - Whether to publish transform to TF tree
  - `initial_tf_yaml_path` (string, default: "") - Path to YAML file with initial transform (optional)

### 4\. frame\_tf\_saver\_node

Looks up a transform from the TF tree and saves it to a YAML file. This is useful for saving calibration transforms or providing initial guesses for ICP.

**Parameters:**

  - `source_frame` (string, default: "morpho\_conveyor/left\_camera/link/realsense\_d435") - Child frame
  - `target_frame` (string, default: "morpho\_conveyor/right\_camera/link/realsense\_d435") - Parent frame
  - `output_yaml_path` (string, default: "camera12\_initial\_tf.yaml") - Output YAML file path
  - `lookup_timeout_sec` (double, default: 10.0) - Maximum time to wait for transform

**Note:** This node runs once, saves the transform, and then shuts down automatically.

## Usage

### Launch Cloud Fusion Node

```bash
ros2 launch multi_cloud_fusion cloud_fusion.launch.py
```

With custom parameters:

```bash
ros2 launch multi_cloud_fusion cloud_fusion.launch.py \
  cloud1_topic:=/camera/front/points \
  cloud2_topic:=/camera/back/points \
  fused_topic:=/merged_output
```

### Launch Top-View Projector Node

```bash
ros2 launch multi_cloud_fusion top_view.launch.py
```

With custom parameters:

```bash
ros2 launch multi_cloud_fusion top_view.launch.py \
  image_width:=800 \
  image_height:=600 \
  x_min:=-2.0 \
  x_max:=2.0 \
  y_min:=-1.5 \
  y_max:=1.5 \
  use_hole_filling:=true
```

### Run Both Nodes Together

```bash
# Terminal 1: Cloud fusion
ros2 launch multi_cloud_fusion cloud_fusion.launch.py

# Terminal 2: Top-view projection
ros2 launch multi_cloud_fusion top_view.launch.py
```

### Topic Remapping

Default topic mappings can be changed in the launch files or via command line:

```bash
ros2 run multi_cloud_fusion cloud_fusion_node \
  --ros-args \
  -r cloud1:=/my/camera1/points \
  -r cloud2:=/my/camera2/points \
  -r fused_cloud:=/my/fused_output
```

### Launch ICP TF Estimation Node

Use this to estimate and publish transforms between two point clouds:

```bash
ros2 launch multi_cloud_fusion icp_tf_node.launch.py
```

With custom parameters:

```bash
ros2 launch multi_cloud_fusion icp_tf_node.launch.py \
  source_topic:=/camera1/points \
  target_topic:=/camera2/points \
  icp_rate_hz:=10.0 \
  voxel_leaf_size:=0.02 \
  fitness_score_threshold:=0.3
```

### Save TF Transform to YAML

Use this to capture and save a transform from the TF tree:

```bash
ros2 launch multi_cloud_fusion frame_tf_saver.launch.py
```

With custom parameters:

```bash
ros2 launch multi_cloud_fusion frame_tf_saver.launch.py \
  source_frame:=camera_left_link \
  target_frame:=camera_right_link \
  output_yaml_path:=/path/to/output/transform.yaml
```

**Typical Workflow for Calibration:**

1.  Use `frame_tf_saver_node` to save an approximate initial transform
2.  Use `icp_tf_node` with the saved transform as `initial_tf_yaml_path` to refine it
3.  Save the refined transform for use in cloud fusion

**YAML Transform File Format:**

```yaml
parent_frame: target_frame_name
child_frame: source_frame_name
translation: [x, y, z]
rotation: [qx, qy, qz, qw]
```

### Dynamic Parameter Reconfiguration

All parameters support runtime modification:

```bash
# Change fusion target frame
ros2 param set /cloud_fusion_node target_frame new_frame

# Adjust top-view ROI
ros2 param set /top_view_projector_node x_min -3.0
ros2 param set /top_view_projector_node x_max 3.0

# Adjust ICP parameters
ros2 param set /icp_tf_node icp_rate_hz 15.0
ros2 param set /icp_tf_node fitness_score_threshold 0.2
```

## Visualizing Results

Use RViz2 to visualize the outputs:

```bash
rviz2
```

Add displays for:

  - `/fused_cloud` - PointCloud2 display
  - `/top_view/rgb` - Image display
  - `/top_view/depth` - Image display

## Processing Pipeline

### CloudFusionNode Pipeline:

1.  **Synchronization:** Time-synchronize two input clouds (ApproximateTime policy).
2.  **Buffer Allocation:** Allocate a single continuous memory block for the merged output.
3.  **Parallel Transform:** Use OpenMP to transform points from both clouds directly into the output buffer (Zero-Copy logic).
4.  **Publish:** Publish the merged raw buffer immediately.

### TopViewProjectorNode Pipeline:

1.  Project 3D points to 2D pixel coordinates
2.  Keep highest/lowest z per pixel (based on `use_max_height`)
3.  (Optional) 4-neighborhood hole filling on RGB and depth
4.  (Optional) Box filter smoothing on RGB and depth
5.  Publish RGB and depth images

## License

Apache-2.0

## Maintainer

Mohamed Abdelkader (mohamedashraf123@gmail.com)