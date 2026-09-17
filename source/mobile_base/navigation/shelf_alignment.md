# Workstation & Shelf Alignment

This module documents the LiDAR-based precision docking and pose correction pipeline (`align.py`) developed for the RoboCup Smart Manufacturing League (SML) 2026 mobile base. The system bridges the gap between coarse Nav2 waypoint navigation and the strict spatial positioning required for arm manipulation at workstation shelves.

---

## 1. Overview & Objectives

While Nav2 navigates the mobile base reliably across the arena, its terminal arrival tolerance ($\pm 0.09\text{ m}$ translation, $\pm 0.09\text{ rad}$ yaw) is too coarse for reliable grasping from workstation shelves. The shelf alignment node executes closed-loop vision/LiDAR refinement directly in front of the target fixture:

* **Tolerance Tightening:** Narrows position error from $\pm 0.09\text{ m}$ down to $\le 0.03\text{ m}$, and angular misalignment from $\pm 0.09\text{ rad}$ down to $\le 0.03\text{ rad}$ ($1.7^\circ$).
* **Closed-Loop Feedback:** Dispatches incremental adjustment goals to Nav2 as standard `NavigateToPose` actions, ensuring full collision safety and path validation.
* **Autonomous Convergence:** Iteratively measures the physical shelf surface, calculates residual offsets, commands micro-adjustments, and concludes with a deterministic creep approach.

---

## 2. Extraction Pipeline Architecture

The alignment pipeline consumes extracted 2D linear features from raw laser range data:

```{mermaid}
flowchart TD
    SCAN["LiDAR Scan: /scan"] --> LLE["laser_line_extraction Node<br/>Split-and-Merge Algorithm"]
    LLE -->|line_segments| BUF["Temporal Buffer<br/>100 Scan Cycles"]
    BUF --> STAGE1["Stage 1: Segment Selection and Stitching"]
    STAGE1 --> STAGE2["Stage 2: Component-wise Averaging"]
    STAGE2 --> STAGE3["Stage 3: Standoff Pose Calculation"]
    STAGE3 --> STAGE4["Stage 4: Measure-Correct Loop<br/>Max 3 Iterations"]
    STAGE4 --> STAGE5["Stage 5: Final Creep Move 0.05 m"]
    STAGE5 --> DONE["Status: Completed"]
```

### Integration with `laser_line_extraction`
* **Input Topic:** Consumes `laser_line_extraction/LineSegmentList` on the `/line_segments` topic.
* **Mechanism:** The upstream extractor executes a split-and-merge line-fitting algorithm across the raw LiDAR point cloud, outputting parameterized line segments with start and end Cartesian endpoints.

---

## 3. Five-Stage Alignment Pipeline

The alignment controller processes laser segments through five distinct stages:

### Stage 1: Segment Selection & Stitching (`_get_workspace_line_segment`)

In real competition environments, shelf edges frequently cause the extractor to split a continuous shelf front into two segments at corner boundaries:

* **Two Segments Detected:** Stitched into a single representative span. Endpoint connectivity is chosen by maximizing:
  $$\max\left(\text{dist}(\text{seg}_0.\text{start}, \text{seg}_1.\text{end}),\; \text{dist}(\text{seg}_1.\text{start}, \text{seg}_0.\text{end})\right)$$
  The wider outer span captures the true physical boundary of the shelf, ignoring internal gaps. The line angles are averaged.
* **One Segment Detected:** Utilized directly as the candidate edge.
* **Zero or $>2$ Segments:** Returns `None` and awaits subsequent scan frames.
* **Geometric Validation Gate:** Candidate segments must pass a rigorous length gate:
  $$L_{\text{shelf}} \in [0.8 - 0.1\text{ m},\; 0.8 + 0.1\text{ m}] = [0.7\text{ m},\; 0.9\text{ m}]$$
  This strict physical filtering prevents the robot from latching onto surrounding walls, adjacent robots, or tote boxes.

### Stage 2: Temporal Averaging (`_get_avg_line_segment`)

To eliminate LiDAR beam noise, range quantization, and extractor jitter, candidate segments passing Stage 1 are collected across a **100-message buffer**. Endpoints and orientations are averaged component-wise before pose calculation:

$$\bar{x}_{\text{start}} = \frac{1}{N}\sum_{i=1}^{N} x_{\text{start}, i}, \qquad \bar{x}_{\text{end}} = \frac{1}{N}\sum_{i=1}^{N} x_{\text{end}, i}$$

### Stage 3: Target Pose Computation (`_finish_alignment`)

The node calculates the geometric midpoint of the averaged line in the `lidar_link` frame:

$$
x_{\text{mid}} = \frac{\bar{x}_{\text{start}} + \bar{x}_{\text{end}}}{2}, \qquad y_{\text{mid}} = \frac{\bar{y}_{\text{start}} + \bar{y}_{\text{end}}}{2}
$$

From this midpoint, residual alignment errors are extracted:
* **Angle Error ($\theta_{\text{err}}$):** Measured angle between the line normal and the robot centerline ($0\text{ rad}$ indicates perpendicular alignment).
* **Lateral Offset ($y_{\text{mid}}$):** Transverse coordinate in `lidar_link` ($0\text{ m}$ indicates centering directly in front of the shelf).

#### Standoff Position Back-Off

To maintain safe arm workspace depth and account for sensor mounting offsets, the target coordinate backs off perpendicular to the shelf face:

$$
d_{\text{total}} = d_{\text{safety}} + x_{\text{base} \to \text{lidar}}
$$

$$
x_{\text{target}} = x_{\text{mid}} - d_{\text{total}}\cos\theta_{\text{err}}, \qquad y_{\text{target}} = y_{\text{mid}} - d_{\text{total}}\sin\theta_{\text{err}}
$$

The resulting Cartesian pose is transformed from `lidar_link` into the global `map` frame via `tf2_ros` (`do_transform_pose`) and submitted as a Nav2 goal.

### Stage 4: Closed-Loop Convergence Loop

The node establishes an iterative **measure $\to$ correct $\to$ measure** loop:

```python
# Convergence verification criteria
if abs(angle_error) <= 0.03 and abs(lateral_offset) <= 0.03:
    self.execute_final_creep()
else:
    if self.iteration_count < self.max_align_iterations:
        self.dispatch_nav2_goal(target_pose_map)
        self.iteration_count += 1
    else:
        self.get_logger().warn("Max alignment iterations reached; continuing best-effort.")
        self.execute_final_creep()
```

* **Iteration Ceiling:** Capped at $3$ iterations (`max_align_iterations = 3`). If the robot fails to converge within 3 attempts, it logs a warning and proceeds best-effort to prevent task stalls.

### Stage 5: Final Creep Maneuver

Once angular and lateral errors converge within $\le 0.03\text{ m}$ and $\le 0.03\text{ rad}$:

* The robot executes a deterministic open-loop forward creep of **$0.05\text{ m}$** along the verified heading.
* Emits lifecycle signal `event_out = "Completed"` to notify the manipulation executive that the arm may safely deploy.

---

## 4. Software Interface & Parameters

### ROS 2 Topics

| Topic Name | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/line_segments` | `laser_line_extraction/LineSegmentList` | Sub | Extracted linear features from the planar LiDAR scan. |
| `/event_in` | `std_msgs/String` | Sub | Command string triggering the alignment routine (`"e_start"`). |
| `/event_out` | `std_msgs/String` | Pub | Status string emitted upon completion (`"Completed"` or `"Failed"`). |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Client | Dispatches micro-correction goals to the Nav2 action server. |

### Configurable Tuning Parameters

| Parameter | Default Value | Description |
| :--- | :--- | :--- |
| `workspace_length` | `0.8 m` | Nominal length of competition workstation shelves. |
| `length_tolerance` | `0.1 m` | Acceptable deviation window ($0.7\text{ m}$ to $0.9\text{ m}$). |
| `target_distance` | `0.35 m` | Desired final standoff distance from shelf front face. |
| `tolerance_xy` | `0.03 m` | Position convergence threshold. |
| `tolerance_yaw` | `0.03 rad` ($\approx 1.7^\circ$) | Angular convergence threshold. |
| `max_align_iterations`| `3` | Maximum correction attempts before fallback creep. |
| `creep_distance` | `0.05 m` | Final forward approach displacement. |
| `buffer_size` | `100` | Number of sequential line segment frames averaged. |
