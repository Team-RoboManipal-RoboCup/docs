# SLAM & Nav2 Motion Planning

This module details the 2D LiDAR simultaneous localization and mapping (SLAM) pipeline, global localization via AMCL, and the Nav2 path planning and trajectory tracking architecture for the RoboCup Smart Manufacturing League (SML) 2026 mobile base.

---

## 1. Overview & Objectives

Autonomous navigation in the SML arena requires both high-precision global mapping and agile, collision-free omnidirectional maneuvering. The mobile base utilizes:

* **SLAM Toolbox:** Generates persistent 2D occupancy grid maps of the competition arena and provides real-time SLAM during dynamic exploration.
* **AMCL Localization:** Fuses LiDAR scan beams with dead-wheel odometry using an omnidirectional motion model (`OmniMotionModel`) to maintain absolute pose within the pre-built map.
* **Nav2 Navigation Stack:** Pairs global path planning via `SmacPlanner2D` with an omnidirectional Model Predictive Path Integral (`MPPI`) local trajectory optimizer.

```{image} ../assets/SLaM.jpeg
:alt: SLAM Global Arena Map
:width: 520px
:align: center
```
*Figure 1: High-resolution occupancy grid map generated using SLAM Toolbox in the arena.*

---

## 2. SLAM Toolbox (Mapping Mode)

The robot uses `slam_toolbox` in online mapping mode to generate arena maps (`.pgm` image and `.yaml` metadata) by fusing planar `/scan` returns with filtered odometry.

### SLAM Configuration Parameters

| Parameter | Configured Value | Functional Role & Rationale |
| :--- | :--- | :--- |
| `use_sim_time` | `true` | Synchronizes node execution with the `/clock` topic in simulation (toggled to `false` on physical hardware). |
| `odom_frame` | `odom` | Input odometry coordinate frame ID published by `smart_brain` / EKF. |
| `map_frame` | `map` | Root reference coordinate frame established by SLAM Toolbox. |
| `base_frame` | `base_footprint` | Projected ground-plane robot base frame used for scan matching. |
| `scan_topic` | `/scan` | Input `sensor_msgs/LaserScan` topic broadcast by the LiDAR sensor driver. |
| `mode` | `mapping` | Operates in active online SLAM mode, continuously publishing `map → odom`. |
| `min_laser_range` | `0.35 m` | Range cutoff discarding returns closer than 35 cm to filter out chassis reflections. |
| `max_laser_range` | `10.0 m` | Maximum scan range threshold to eliminate noisy far-field points. |
| `transform_timeout`| `1.0 s` | Maximum allowable latency for TF buffer lookups before scan dropping occurs. |
| `tf_buffer_duration`| `30.0 s` | Retained transform history duration for scan interpolation across network jitter. |
| `throttle_scans` | `1` | Ingests every laser scan without skipping for maximum map fidelity. |

---

## 3. Nav2 Navigation Stack Architecture

### 3.1 Coordinate Frames (TF Tree)

Global localization and motion execution are structured along a standard hierarchical transform tree:

```
map  ──(AMCL)──>  odom  ──(smart_brain / EKF)──>  base_link  ──(static)──>  base_footprint
```

* **`map → odom`**: Computed and broadcast dynamically by AMCL to correct accumulated odometry drift.
* **`odom → base_link`**: Continuously broadcast at 100 Hz by the dead-wheel odometry pipeline.
* **`base_link → base_footprint`**: Static mechanical transform linking chassis center to ground projection.

### 3.2 Control Pipeline

```
Global Navigation Goal
        │
        ▼
planner_server (SmacPlanner2D, global_costmap: map frame)
        │  Global Path
        ▼
smoother_server (SimpleSmoother)
        │  Smoothed Path
        ▼
controller_server (MPPI FollowPath, local_costmap: odom frame, rolling window)
        │  cmd_vel (unfiltered omni commands)
        ▼
velocity_smoother
        │  cmd_vel (rate- and acceleration-limited)
        ▼
Base Motor Controllers (STM32 Mecanum Drive)
```

---

## 4. Localization: Adaptive Monte Carlo Localization (AMCL)

AMCL tracks the robot's pose against the pre-built static map using an omnidirectional motion model:

| Parameter | Value | Functional Description |
| :--- | :--- | :--- |
| `robot_model_type` | `nav2_amcl::OmniMotionModel` | Specifically matches the 4-wheel mecanum omnidirectional kinematics (replaces default diff-drive). |
| `alpha1` – `alpha5` | `0.2` each | Noise parameters across rotation, translation, and strafe motions. Uniform 0.2 values accommodate omnidirectional dead-wheel drift characteristics. |
| `min_particles` / `max_particles` | `1000` / `5000` | Particle bounds balancing localization convergence in symmetric arenas against compute overhead. |
| `laser_model_type` | `likelihood_field` | Computes beam-endpoint likelihood against precalculated distance fields for fast evaluation. |
| `laser_min_range` / `laser_max_range` | `-1.0` / `100.0` | Defers to sensor-reported bounds (`-1.0` uses LiDAR native minimum). |
| `max_beams` | `60` | Subsampled scan beams per evaluation to cap compute cost per particle filter cycle. |
| `update_min_d` / `update_min_a` | `0.1 m` / `0.1 rad` | Minimum translational and angular displacement thresholds required to trigger a particle update. |
| `resample_interval` | `1` | Resamples particles on every filter update cycle without delay. |
| `recovery_alpha_slow` / `fast` | `0.0` / `0.0` | Random particle injection disabled; localization relies on `set_initial_pose` seeds. |
| `set_initial_pose` | `true` | Seeds initial belief at startup coordinates `(0, 0, 0.0165 rad)`, bypassing global search. |
| `tf_broadcast` | `true` | AMCL directly publishes the `map → odom` coordinate transform. |
| `transform_tolerance` | `0.5 s` | Transform lookahead tolerance to accommodate consumer latency. |

---

## 5. Local Controller: MPPI (Model Predictive Path Integral)

The local controller uses the Model Predictive Path Integral (`MPPI`) algorithm configured for omnidirectional trajectory generation.

### 5.1 Controller Parameters

| Parameter | Value | Description |
| :--- | :--- | :--- |
| `motion_model` | `Omni` | Evaluates forward ($v_x$), lateral ($v_y$), and angular ($\omega_z$) velocities simultaneously. |
| `time_steps` / `model_dt` | `56` / `0.05 s` | Rollout horizon $\approx 2.8\text{ s}$ evaluated at 20 Hz control rate. |
| `batch_size` | `2000` | Number of candidate trajectory rollouts evaluated per control cycle. |
| `vx_max` / `vx_min` | `1.0 m/s` / `-0.5 m/s` | Longitudinal velocity sampling bounds (asymmetric forward bias). |
| `vy_max` | `1.0 m/s` | Lateral strafing velocity limit. |
| `wz_max` | `1.0 rad/s` | Maximum rotational velocity. |
| `vx_std` / `vy_std` / `wz_std` | `0.5` / `0.5` / `0.2` | Sampling noise distribution standard deviations for trajectory generation. |
| `iteration_count` | `1` | Single optimization pass per control cycle for guaranteed real-time determinism. |
| `temperature` / `gamma` | `0.15` / `0.015` | Trajectory softmax sharpness weighting and control cost regularization factor. |
| `controller_frequency`| `20.0 Hz` | Control loop update rate. |

### 5.2 Active Trajectory Critics

Trajectories are scored across multiple competing criteria:

| Critic Name | Weight & Key Settings | Functional Objective |
| :--- | :--- | :--- |
| **`ConstraintCritic`** | `4.0` | Penalizes trajectories that violate chassis velocity and acceleration limits. |
| **`GoalCritic`** | `5.0` (`threshold_to_consider: 0.8 m`) | Pulls trajectory toward goal position once within 0.8 m proximity. |
| **`GoalAngleCritic`** | `8.0` (`threshold_to_consider: 1.4 m`) | Aligns robot heading with the terminal goal orientation. |
| **`ObstaclesCritic`** | Repulsion: `6.0`, Critical: `20.0`<br/>(`collision_margin: 0.2 m`) | Footprint-aware collision avoidance; hard-rejects paths inside safety margins. |
| **`CostCritic`** | `6.0` (`critical_cost: 400`) | Discourages motion through high-cost regions of the inflated costmap. |
| **`PathAlignCritic`** | `3.0` (`offset_from_furthest: 4`) | Minimizes lateral deviation from the global planned path. |
| **`PathFollowCritic`**| `3.0` (`offset_from_furthest: 5`) | Encourages forward progress along the path toward lookahead points. |
| **`PathAngleCritic`** | `1.0` (`mode: 2`) | Penalizes heading discrepancies relative to the path tangent vector. |

:::{note}
`PreferForwardCritic` and `TwirlingCritic` are deliberately **disabled**. This allows the omnidirectional Mecanum base to strafe laterally and rotate freely without artificial penalties, maximizing maneuverability in tight warehouse aisles.
:::

---

## 6. Global Planner & Auxiliary Servers

### 6.1 SmacPlanner2D (`GridBased`)
* **`tolerance`**: `0.15 m` — Maximum allowable Euclidean deviation if the target cell is occupied.
* **`allow_unknown`**: `true` — Enables path exploration across partially mapped regions.
* **`max_iterations`**: `1,000,000` — Search budget for the 2D A* solver before failing.
* **`cost_travel_multiplier`**: `5.0` — Heavily penalizes proximity to obstacles, biasing paths toward the geometric center of corridors.
* **`smooth_path`**: `true` — Pre-smooths raw grid-based waypoint sequences.

### 6.2 Supporting Servers
* **`smoother_server`**: Implements `SimpleSmoother` with numerical tolerance of $10^{-10}$ over up to 1000 iterations.
* **`behavior_server`**: Manages recovery behaviors (`spin`, `backup`, `drive_on_heading`, `wait`) operating in the `odom` frame with a 2.0 s collision lookahead.
* **`waypoint_follower`**: Integrates `WaitAtWaypoint` with a 200 ms dwell time at intermediate stations.
* **`velocity_smoother`**: Shapes output commands with acceleration limits (`[3.2, 3.2, 4.0]` $\text{m/s}^2$ / $\text{rad/s}^2$) in `OPEN_LOOP` mode.

---

## 7. Costmap Layering & Geometry

The navigation stack maintains dual 2D costmaps sharing an exact polygonal footprint:

* **Robot Footprint:** `[[0.31, 0.29], [0.31, -0.29], [-0.31, -0.29], [-0.31, 0.29]]`  
  A $0.62\text{ m} \times 0.58\text{ m}$ bounding rectangle matching the mechanical chassis. Critics evaluate true polygon collision models rather than approximate radius circles.

| Costmap | Reference Frame | Window Extent | Update Rate | Layer Stack & Parameters |
| :--- | :--- | :--- | :--- | :--- |
| **Local Costmap** | `odom` | $4.0\text{ m} \times 4.0\text{ m}$ (Rolling) | 5.0 Hz | `voxel_layer` (live LiDAR scan, raytrace 3.0 m) + `inflation_layer` (`inflation_radius: 0.27 m`) |
| **Global Costmap** | `map` | Full arena extent | 1.0 Hz | `static_layer` (SLAM occupancy grid) + `obstacle_layer` (live LiDAR) + `inflation_layer` (`inflation_radius: 0.45 m`) |

---

## 8. Software & Hardware Topics

| Topic Name | Message Type | Direction | Associated Nodes | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/LaserScan` | Sub | AMCL, Costmaps, SLAM | Raw planar LiDAR range scans. |
| `/odometry/filtered` | `nav_msgs/Odometry` | Sub | `bt_navigator`, `velocity_smoother` | Fused state estimation from the EKF filter. |
| `/map` | `nav_msgs/OccupancyGrid` | Pub / Sub | `map_server`, AMCL, Costmaps | Global arena occupancy grid. |
| `/cmd_vel` | `geometry_msgs/Twist` | Pub | `velocity_smoother` → Motor Driver | Final velocity commands driving the Mecanum wheels. |
| `local_costmap/costmap_raw` | `nav2_msgs/Costmap` | Sub | `behavior_server` | Real-time rolling obstacle map for recovery checks. |

---

## 9. Hardware Deployment Checklist

When transitioning from the Gazebo simulation environment to physical robot hardware:

1. **Clock Synchronization:** Set `use_sim_time: false` across all node configurations (`amcl`, `controller_server`, `planner_server`, `behavior_server`, `map_server`).
2. **Initial Pose Seeding:** AMCL kidnapping recovery is disabled (`recovery_alpha_* = 0.0`); ensure startup coordinates match arena initialization or dispatch an initial pose via RViz2 / task executive.
3. **Friction Calibration:** Surface friction on competition tiles may require tuning MPPI `vx_max` and acceleration limits in `velocity_smoother`.
4. **Closed-Loop Velocity Feedback:** If commanded velocities show divergence against true chassis response, toggle `velocity_smoother` feedback to `CLOSED_LOOP`.
