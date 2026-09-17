# Object Detection & Pose Estimation (PCL)

Averaged 6-DoF pose estimation pipeline for RoboCup SML brick manipulation. Fuses YOLO oriented-bounding-box (OBB)(`inference_node`) detections with an organized RGB-D point cloud (processed `pcl_geometry_node`), then temporally filters and averages per-block poses over `N` frames through a ROS 2 service (`perception_server`) before handing a stable grasp/place target to the manipulation stack.

---

## 1. Overview & Objectives

- **Problem solved:** single-frame PCA-based pose-estimation suffer sign-flip instability across frames due to noise and stray foreground pixels. This pipeline replaces raw single-shot pose estimates with a RANSAC-plane + minAreaRect hybrid orientation estimation, then averages `target_samples` consistent frames before sending a pose to the arm.
- **Two operating modes**, can be toggled at runtime rather than requiring separate nodes:
  1. **Pick mode** (`GET_AVG_PICK_POSES`): Detects individual bricks, returns per-block averaged pose, class, ID, and OBB dimensions. Also ignores any block that is inconsistent across the `target_samples` number of frames. 
  2. **Place mode** (`GET_AVG_PLACE_POSES`): Bypasses detection part of the callback completely and computes an evenly spaced row of drop-off poses across the workbench's cloud's bounding extent, so that the warehouse bot can safely drop off the required material at the workstation for the workbench arm to assemble further.
- **Consumer:** a single synchronous service call (`/perception/get_averaged_pose` or `/perception/get_averaged_place_pose`) returns averaged data directly to the motion-planning client. No subscription of raw topics is required by downstream nodes as the `pcl_geometry_node` is computationally heavy to run.
- **Camera/sim portability:** all topic names and RANSAC/cluster tuning live in one file, `config/camera_config.yaml`, loaded by `camera.launch.py` Switching between ZED hardware and a simulated RGB-D source (or any future camera) means editing that file only, never the launch file or node source (see {ref}`4.4 <section-launch-config>`).

---

## 2. Technical Specifications & Bill of Materials

### 2.1 Node / Package Composition

| Node (executable) | Package | Responsibility |
| :--- | :--- | :--- |
| `inference_node` | `gazebo_perception` | YOLO OBB detection + tracking on RGB stream |
| `pcl_geometry_node` | `pcl_geometry` | Point-cloud pre-processing, RANSAC based plane removal, clustering, hybrid orientation, place and pick pose generation |
| `perception_server` | `gazebo_perception` | Multi-frame collection and averaging, consistency filtering by collecting detection over multiple frames, position/quaternion averaging, acts as a front-end for all the available services|
| `visualizer_node` | `gazebo_perception` | RViz debug visualization (hardware launch only) |

### 2.2 `inference_node` (YOLO) Parameters

| Parameter | Default | Notes |
| :--- | :--- | :--- |
| `model_path` | `<pkg_share>/weights/best.pt` | Resolved via `get_package_share_directory("gazebo_perception")` |
| `camera_input_topic` | `/rgb_camera/image_raw` | Overridden to `/zed/zed_node/rgb/color/rect/image` on hardware. Set via `inference_node.ros__parameters` in `camera_config.yaml` |
| `annotated_image_topic` | `/vision/annotated_image` | The annotated YOLO detection are published to this topic for debugging/tuning |
| `detection_topic` | `/vision/detected_objects` | `vision_msgs/msg/Detection2DArray` |
| Inference call | `model.track(cv_image, persist=True, conf=0.7)` | OBB tracking (not plain detect), assigns persistent `box.id` |

### 2.3 `pcl_geometry_node` Tunable Parameters

| Parameter | Default | Unit / Meaning |
| :--- | :--- | :--- |
| `point_cloud_topic` | `/rgb_camera/points` | Overridden to `/zed/zed_node/point_cloud/cloud_registered` on hardware by default.Set via `pcl_geometry_node.ros__parameters` in `camera_config.yaml` for other hardware|
| `ransac_distance_threshold` | `0.008` | m - plane inlier distance for `SACMODEL_PERPENDICULAR_PLANE` |
| `ransac_max_iterations` | `1000` | RANSAC iteration cap |
| `ransac_eps_angle_deg` | `4.0` | ° - max deviation of fitted plane normal from world Z |
| `cluster_tolerance` | `0.01` | m - Euclidean cluster point spacing |
| `min_cluster_size` / `max_cluster_size` | `30` / `500` | pts - valid brick cluster bounds |
| `margin_px` | `8.0` | px - dilation of the OBB mask beyond the tight detection rectangle |
| `min_cloud_size` | `10` | pts - reject ROI before RANSAC/clustering if the number of points in the cluster are below this |

All eight are tunable at run-time via `add_on_set_parameters_callback` . No node restart needed for RANSAC/cluster tuning during calibration.

(section-place-constants)=
### 2.4 Place-Pose Parameters (place mode)

| Parameter | Default | Meaning |
| :--- | :--- | :--- |
| `max_arm_reach` / `min_arm_reach` | `0.1` / `-0.1` m(default values) | Lateral span (`max_y`/`min_y` in code) over which place poses are distributed (map frame). Tune per arm/workbench reachability. Declared as ROS 2 parameters, settable via `camera_config.yaml` |
| `place_pose_z_offset` | `0.05` m | Added above measured surface `z` (`z_offset` in code) to avoid collision on placement, since a point-cloud surface/hull centroid alone doesn't give a reliable placement height. Declared as a ROS 2 parameter, settable via `camera_config.yaml` |

### 2.5 Collection Timing (`perception_server`)

| Constant | Value | Meaning |
| :--- | :--- | :--- |
| `TIMEOUT_COUNT` | `50` | timer ticks |
| Timer period | `TIMEOUT_COUNT * 0.10` = **5.0 s** | Hard cap on frame-collection wait before returning whatever was gathered |

---

## 3. Design & Architecture

### 3.1 Pipeline Flow

```{mermaid}
flowchart LR
    A["inference_node<br/>YOLO OBB track"] -->|Detection2DArray| C["pcl_geometry_node<br/>ApproximateTime Sync"]
    B["RGB-D Source<br/>Depth Camera"] -->|PointCloud2| C
    C -->|PerceptionMsg| D["perception_server"]
    C -->|PlacePoseMsg| D
    D -->|Planning Request| E["Motion Planning Client"]
    D -->|Toggle Pipeline| C
```

`pc_sub_` and `det_sub_` are synchronized with `message_filters::sync_policies::ApproximateTime`, queue size **50**, `setMaxIntervalDuration(0.1 s)`. Detection and cloud frames more than 100 ms apart are never paired.

(section-roi)=
### 3.2 ROI Extraction (per detection)

1. Detection `bbox` (center, size, theta from OBB) is rescaled from the 1280×720 YOLO inference resolution to the actual cloud resolution:

   $$ \text{scale}_x = \frac{W_{cloud}}{1280}, \quad \text{scale}_y = \frac{H_{cloud}}{720} $$

2. Two `cv::RotatedRect` masks are taken per detection: a **tight** mask (exact OBB) and an **expanded** mask (`+2·margin_px` on each side, default 8 px).
3. Cloud lookup assumes an **organized** `PointCloud2`: `memory_index = v * width + u`. Points failing `std::isfinite` on x/y/z are dropped at this stage.
4. The tight-mask cloud's centroid becomes `reference_point` which is used later to distinguish which Euclidean cluster is this brick when adjacent bricks bleed into the expanded mask.

:::{warning}
The organized-cloud indexing assumption (`v * width + u`) will silently return garbage points if fed an unorganized/dense-filtered `PointCloud2`. Verify `pc_msg->height > 1` before wiring a new camera/sensor.
:::

:::{figure} assets/normal_pointcloud.png
:alt: Raw RGB-D point cloud input
:width: 60%

Raw RGB-D point cloud input showing the workspace prior to YOLO OBB fusion and processing.
:::

### 3.3 Table Removal & Clustering

- `pcl::SACSegmentation<PointXYZRGB>` with `SACMODEL_PERPENDICULAR_PLANE`, axis locked to world `+Z`, `eps_angle = ransac_eps_angle_deg`(This is error angle and any plane is valid if it falls within this error range). Inliers (table) are extracted with `setNegative(true)` to leave only above-table geometry.
- `pcl::EuclideanClusterExtraction` on the finite-filtered remainder. When `have_reference_point` is true, the cluster whose centroid is nearest (squared distance) to `reference_point` is selected.

:::::{grid} 2

::::{grid-item}
:::{figure} assets/ransac_cloud.png
:alt: Point cloud segments extracted using YOLO OBB
:width: 100%

Point cloud segments extracted using the scaled YOLO bounding box masks prior to table plane removal.
:::
::::

::::{grid-item}
:::{figure} assets/extracted_clusters.png
:alt: Isolated brick clusters
:width: 100%

Isolated brick clusters remaining after RANSAC perpendicular plane removal and Euclidean clustering.
:::
::::

:::::

### 3.4 Hybrid Orientation Estimation

`estimateHybridOrientation()` builds a right-handed rotation frame per brick rather than trusting raw PCA (which is unstable to eigenvector sign flips across frames):

- **Z-axis**: table plane normal from RANSAC, forced to `+Z` hemisphere (`if (table_normal.z() < 0) table_normal = -table_normal`).
- **X-axis**: longer edge of `cv::minAreaRect` over the brick cloud's 2D (x,y) projection, Gram-Schmidt orthogonalized against `V_z`:

  $$ V_x = \frac{\vec{e} - (\vec{e}\cdot V_z)\,V_z}{\lVert \vec{e} - (\vec{e}\cdot V_z)\,V_z \rVert} $$

  Falls back to world `+X` if the projected edge is near-parallel to `V_z` (`‖·‖ < 1e-4`). Sign-locked so `V_x.x() ≥ 0`. Prevents 180°-flipped grasp orientations between frames.
- **Y-axis**: completes the right-handed frame: $V_y = V_z \times V_x$.
- Final quaternion built directly from the rotation matrix `[V_x | V_y | V_z]`.

:::{figure} assets/pose_estimation.jpeg
:alt: Hybrid orientation axes overlaid on detected bricks
:width: 60%

RViz TF axes ($V_x$, $V_y$, $V_z$) overlaid on detected bricks — long edge ($V_x$, red) aligned to the OBB's longer side, table normal ($V_z$, blue) pointing up.
:::

### 3.5 Place-Pose Generation

In place mode the node skips detection/RANSAC entirely, transforms the whole cloud `optical → map`, then evenly distributes `number_of_places` poses across the fixed lateral band:

$$ d = \frac{y_{max} - y_{min}}{N + 1}, \qquad y_i = y_{min} + i \cdot d \quad \text{for } i = 1 \ldots N $$

with `x = x_mid` (bounding-box midpoint) and `z = z_{avg} + z_{offset}` (cloud centroid Z + 0.05 m clearance). All place poses carry identity orientation.

:::::{grid} 2

::::{grid-item}
:::{figure} assets/place_pose_0.1m.png
:alt: Place poses distributed with a 0.1m lateral span
:width: 100%

Place poses distributed with a 0.1m lateral span.
:::
::::

::::{grid-item}
:::{figure} assets/place_poses_0.3m.png
:alt: Place poses distributed with a 0.3m lateral span
:width: 100%

Place poses distributed with a 0.3m lateral span.
:::
::::

:::::

### 3.6 Multi-Frame Averaging (`perception_server`)

1. **Consistency filter**: frames are sorted by descending detected-block count; the largest frame's `block_ids` set becomes the reference `check_list`. Any frame whose IDs aren't a subset of `check_list` is discarded before averaging (rejects false-positive detections or detections that are not stable enough across `target_samples` number of frames).
2. **Position averaging**: simple per-axis mean across valid frames for each surviving `block_id`.
3. **Quaternion averaging**: naive component-wise mean is only valid after resolving double-cover: each sample quaternion is dot-producted against the *first* collected sample (`q_ref`); if `dot < 0` the sample is negated before averaging, then the mean is renormalized:

   $$ \bar{q} = \frac{1}{n}\sum_i \text{sign}(q_i \cdot q_{ref})\, q_i, \qquad \hat{q} = \frac{\bar{q}}{\lVert \bar{q} \rVert} $$

4. **Dimension averaging**: per-axis mean of `SolidPrimitive.BOX_X/Y/Z` extents across the same surviving frame set.
5. Same sign-correction + averaging logic is duplicated in `get_avg_place_poses()` for the place-pose path (per-index averaging across `number_of_places` slots instead of per block-ID as these are the empty places where any valid block can be placed).

:::{tip}
Both averaging loops key off the *first* frame collected, not a running reference. If the first frame happens to be a partial/edge detection, its `block_ids` order still gates the whole batch. Worth checking `collected_frames.sort()` (descending count) is doing its intended job before trusting a service response with few samples.
:::

---

## 4. Software/Hardware Interface

### 4.1 Topics

| Topic | Type | Pub → Sub | Notes |
| :--- | :--- | :--- | :--- |
| `/rgb_camera/image_raw` (sim) / `/zed/zed_node/rgb/color/rect/image` (hw) | `sensor_msgs/msg/Image` | camera → `inference_node` | resolved via `camera_input_topic` param |
| `/vision/detected_objects` | `vision_msgs/msg/Detection2DArray` | `inference_node` → `pcl_geometry_node` | OBB center/size/theta packed into `bbox` |
| `/vision/annotated_image` | `sensor_msgs/msg/Image` | `inference_node` → debug viewer | annotated images are published to this topic for debugging/tuning the model used for object recognition|
| `/rgb_camera/points` (sim) / `/zed/zed_node/point_cloud/cloud_registered` (hw) | `sensor_msgs/msg/PointCloud2` | camera/`depth_image_proc` → `pcl_geometry_node` | resolved via `point_cloud_topic` param; must be **organized** |
| `/rgbd_camera/image`, `/rgbd_camera/depth_image`, `/rgbd_camera/camera_info` | `sensor_msgs/msg/Image`, `CameraInfo` | Gazebo bridge → `PointCloudXyzrgbNode` | sim-only, from `ros_gz_bridge parameter_bridge` |
| `/zed/zed_node/rgb/color/rect/image/camera_info` | `sensor_msgs/msg/CameraInfo` | ZED → `pcl_geometry_node` | populates `K_matrix_` once (`has_k_matrix_` latch that turns `true` whenever it gets the first valid message) |
| `/perception/brick_data` | `perception_msgs/msg/PerceptionMsg` | `pcl_geometry_node` → `perception_server` | pick-mode per-frame detections |
| `/perception/place_poses` | `perception_msgs/msg/PlacePoseMsg` | `pcl_geometry_node` → `perception_server` | place-mode per-frame candidate poses |
| `/debug/brick_poses` | `geometry_msgs/msg/PoseArray` | `pcl_geometry_node` → RViz | the raw poses generated by the `pcl_geometry_mode` |
| `/debug/points`, `/debug/ransac` | `sensor_msgs/msg/PointCloud2` | `pcl_geometry_node` → RViz | post-ROI cloud, post-plane-removal cloud |
| `/debug/obb_boxes` | `visualization_msgs/msg/MarkerArray` | `pcl_geometry_node` → RViz | `CUBE` markers, RGBA `(0,1,1,0.5)` |

### 4.2 Services

| Service | Type | Server | Request → Response |
| :--- | :--- | :--- | :--- |
| `/perception/get_averaged_pose` | `perception_msgs/srv/PerceptionSrv` | `perception_server` | `int32 target_samples` → `block_poses`, `block_ids`, `block_classes`, `block_dims` |
| `/perception/get_averaged_place_pose` | `perception_msgs/srv/PlacePoseSrv` | `perception_server` | `uint8 target_positions`, `uint8 target_samples` → `place_poses` |
| `/perception/set_active` | `std_srvs/srv/SetBool` | `pcl_geometry_node` | called internally by `perception_server` to arm/disarm pick-mode detection |
| `/perception/set_place_active` | `perception_msgs/srv/SetPlaceToggle` | `pcl_geometry_node` | `bool data, uint8 target_positions` → toggles place mode; also forces `active_ = false` |

`perception_server` runs both services on a `ReentrantCallbackGroup`; `pcl_geometry_node` splits processing / service / publishing across three **mutually-exclusive** groups on a 4-thread `MultiThreadedExecutor`. The sync callback and the toggle services cannot preempt each other.

### 4.3 Message/Service Definitions

```{code-block} text
:caption: PerceptionMsg.msg

geometry_msgs/PoseArray block_poses
string[] block_ids
string[] block_classes
shape_msgs/SolidPrimitive[] block_dims
```

```{code-block} text
:caption: PlacePoseMsg.msg

geometry_msgs/PoseArray place_poses
```

```{code-block} text
:caption: PerceptionSrv.srv

int32 target_samples
---
geometry_msgs/PoseArray block_poses
string[] block_classes
string[] block_ids
shape_msgs/SolidPrimitive[] block_dims
geometry_msgs/PoseArray place_poses
```

```{code-block} text
:caption: PlacePoseSrv.srv

uint8 target_positions
uint8 target_samples
---
geometry_msgs/PoseArray place_poses
```

```{code-block} text
:caption: SetPlaceToggle.srv

bool data
uint8 target_positions
---
bool success
string message
```


(section-launch-config)=
### 4.4 Launch Configurations

| Launch file | Environment | Key differences |
| :--- | :--- | :--- |
| `gazebo_brick.launch.py` | Gazebo Harmonic sim | Starts `gz_sim`, `ros_gz_bridge parameter_bridge` for `/clock`, RGB-D image/depth/camera_info; `depth_image_proc::PointCloudXyzrgbNode` composed into `image_proc_container` to synthesize `/rgbd_camera/points`. Static TF `map → camera_rig/camera_link/rgbd_camera` (roll `-2.35619` rad) |
| `camera.launch.py` | Physical camera (ZED2 by default) | Config-driven. See below. `inference_node`/`pcl_geometry_node` parameters and topic names come entirely from `config/camera_config.yaml`. Adds `visualizer_node` and `rviz2` |

Both launch files set `__NV_PRIME_RENDER_OFFLOAD=1` / `__GLX_VENDOR_LIBRARY_NAME=nvidia`. Required for GPU-accelerated rendering (Gazebo sim) and CUDA-based YOLO inference on appropriate devices

#### 4.4.1 `camera_config.yaml`

Switching to a different physical camera or resim target means editing **only this file**. No changes to `camera.launch.py` or node source.

```{code-block} yaml
:caption: config/camera_config.yaml

inference_node:
  ros__parameters:
    camera_input_topic: "/zed/zed_node/rgb/color/rect/image"
    annotated_image_topic: "/vision/annotated_image"
    detection_topic: "/vision/detected_objects"
    # model_path: "/absolute/path/to/custom_weights.pt"  # uncomment to override the package-default weights/best.pt

pcl_geometry_node:
  ros__parameters:
    point_cloud_topic: "/zed/zed_node/point_cloud/cloud_registered"
    ransac_distance_threshold: 0.008
    ransac_max_iterations: 1000
    ransac_eps_angle_deg: 4.0
    cluster_tolerance: 0.01
    min_cluster_size: 30
    max_cluster_size: 500
    margin_px: 8.0
    min_cloud_size: 10
    max_arm_reach: 0.1
    min_arm_reach: -0.1
    place_pose_z_offset: 0.05
  # Applied via topic remapping in camera.launch.py, not a declared parameter:
  camera_info_topic: "/zed/zed_node/rgb/color/rect/image/camera_info"

static_transform:
  parent_frame: "map"
  child_frame: "zed_camera_link"
  translation:
    x: 0.0
    y: 0.0
    z: 0.57
  rotation_rpy_deg:
    roll: 0.0
    pitch: 51.0
    yaw: 0.0
```

- The `inference_node` and `pcl_geometry_node` blocks are passed straight through to their nodes as a ROS 2 parameters file (`parameters=[config_file]`). Every key under `ros__parameters`, including `max_arm_reach`/`min_arm_reach`/`place_pose_z_offset`, is a real, declared parameter in the node source ({ref}`§2.4 <section-place-constants>`) and tunable live during run-time via the same `add_on_set_parameters_callback` as the RANSAC/cluster params.
- `camera_info_topic` is the one remaining exception. **Not** a declared parameter (`pcl_geometry_node.cpp` subscribes to a hardcoded string for the K-matrix). `camera.launch.py` reads this key itself and applies it as a topic **remap**, so it's still fully configurable from this file without a code change.
- `static_transform` is read directly by the launch file (not passed to any node as a parameter) and converted from degrees to radians before being handed to `static_transform_publisher` as CLI arguments.

#### 4.4.2 `camera.launch.py` Arguments

| Launch argument | Default | Purpose |
| :--- | :--- | :--- |
| `config_file` | `<gazebo_perception share>/config/camera_config.yaml` | Path to the YAML above. Override with `config_file:=/path/to/other.yaml` to keep multiple camera profiles side by side |
| `publish_static_tf` | `false` | Publishes the `static_transform` block via `static_transform_publisher`. Off by default. Set `publish_static_tf:=true` on the CLI if no `robot_state_publisher`/URDF is providing that frame |

```{code-block} bash
:caption: Example: Override both the config file and enable the static TF

ros2 launch gazebo_perception camera.launch.py \
    config_file:=/home/user/configs/oakd_config.yaml \
    publish_static_tf:=true
```

---

## 5. Future Work / Known Limitations

- **Fixed place-pose orientation:**: All generated place poses use identity quaternion. No support yet for orienting placement to match brick footprint or a target assembly pattern.
- **Sync window not adaptive:** `message_filters` `ApproximateTime` max interval is fixed at 0.1 s. Under YOLO inference latency spikes, valid cloud/detection pairs can be dropped rather than the node adapting.
- **No validation of `target_positions` vs. generated pose count:** `get_avg_place_poses()` silently skips a place pose with a `RCLCPP_WARN` if no frame contains exactly `number_of_places` poses, rather than the reporting partial-result count to the caller.
- **Organized-cloud assumption is unchecked at runtime** : No `pc_msg->height` guard before the `v * width + u` loop (see {ref}`3.2 <section-roi>`).
