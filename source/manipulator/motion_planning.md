# Motion Planning (MoveIt 2)

This module manages the system-level integration of a 5 DOF manipulator mounted on an omnidirectional mecanum base for the RoboManipal RoboCup SML 2026 platform. It unifies MoveIt 2 (via MoveIt Task Constructor), TRAC-IK kinematics, and Gazebo Ignition simulations to execute dynamic, collision-aware pick-and-place tasks across varied warehouse workstations.

---

## 1. Overview & Objectives

The integration bridges high-level behavior planning with low-level execution by fusing base navigation, dynamic TF tracking, and task-space manipulator control. Its primary objective is to reliably accept `/arm_command` action goals, dynamically generate obstacle-free trajectories (avoiding sibling blocks and station geometry), and command joint state trajectories to the simulated/real hardware while maintaining continuous odometry and EKF localization.

---

## 2. Technical Specifications & Bill of Materials (BOM)

### 2.1 Kinematics Solver: TRAC-IK
The default KDL solver is replaced with TRAC-IK via the `trac_ik_kinematics_plugin`. TRAC-IK executes two IK algorithms concurrently: a KDL-based Newton-Raphson solver and a Sequential Quadratic Programming (SQP) solver.

**Why TRAC-IK is Superior for 5 DOF Arms:**
A 5 DOF arm is underactuated in 6D space; it physically cannot achieve arbitrary end-effector poses (XYZ + Roll/Pitch/Yaw) simultaneously. The standard KDL pseudo-inverse Jacobian solver attempts exact mathematical convergence for all 6 dimensions, frequently oscillating or getting trapped in local minima. TRAC-IK's SQP optimization natively handles constraints and tolerances, finding the closest possible joint configuration that satisfies positional constraints without failing due to unreachable orientations.

| Feature | KDL Solver | TRAC-IK Solver |
| :--- | :--- | :--- |
| **Algorithm Base** | Pseudo-inverse Jacobian (Newton-Raphson) | Concurrent Newton-Raphson & SQP (NLOPT) |
| **5 DOF Handling** | Frequently fails; gets stuck in local minima trying to resolve 6D constraints. | High success rate; SQP gracefully minimizes error within reachable subset. |
| **Timeouts** | Often exhausts timeout without solution. | Aborts early if trapped, relies on concurrent thread for solution. |
| **Configuration** | Default MoveIt standard. | Requires custom `kinematics.yaml` binding. |

### 2.2 Motion Planning Algorithms
The MoveIt Task Constructor (MTC) node uses a multi-solver approach to handle different stages of the pick-and-place pipeline:
*   **OMPL (TRRT):** The `PipelinePlanner` is configured to use the Transition-based Rapidly-exploring Random Tree (TRRT) algorithm. It is assigned a 20.0-second timeout. TRRT is highly effective in constrained environments, such as reaching into the tight shelves defined in the workstation geometry.
*   **Cartesian Path Planner:** Used for precision approach and lifting maneuvers. It computes linear end-effector trajectories with a step size of 0.01 meters. To ensure smooth hardware execution and prevent slip or dropped payloads, both maximum velocity and maximum acceleration are scaled down to 30% (`0.3`) of the robot's physical limits.
*   **Joint Interpolation Planner:** Used for simple, direct actions where collision checking isn't complex, such as commanding the gripper group to "open" or "close".

### 2.3 Hardware Abstraction (`ros2_control`)
The integration relies on `ros2_control` to bridge joint trajectories to both the Ignition simulation and the real hardware. The launch sequence uses delayed timer actions to ensure stable initialization:
*   `arm_controller` and `gripper_controller` are spawned at 4.0 seconds.
*   `joint_state_broadcaster` is initialized at 15.0 seconds to publish `/joint_states` to the TF tree.
*   `mecanum_controller` is spawned at 18.0 seconds to handle the omnidirectional base commands.

### 2.4 Simulation Environment (Gazebo Ignition) Bridges
Gazebo Ignition uses an isolated messaging system (Ignition Transport). To expose simulator data to the ROS 2 ecosystem, `gazebo.launch.py` utilizes the `ros_gz_bridge` package. This bridge handles bi-directional serialization and deserialization of messages across the middleware barrier.

The exact bridge configuration maps vital sensor pipelines:
```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='bridge',
    output='screen',
    arguments=[
        # Map Gazebo Clock to ROS 2 /clock for use_sim_time synchronization
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        
        # Stream raw IMU data for robot_localization EKF
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        
        # Expose absolute ground-truth model poses (consumed by gazebo_station_relay)
        '/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        
        # 2D Lidar for mapping and line_extraction alignment
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        
        # RGB-D camera pipelines
        '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ],
    parameters=[{'use_sim_time': True}],
)
```

---

## 3. Design & Architecture

### 3.1 Task Planning & Execution (AI Planning)
The high-level logic is governed by two dedicated nodes utilizing the `unified_planning` (UP) framework:

*   **TaskPlannerNode (`task_planner_node.py`):** Acts as the brain of the operation. It receives a master list of `orders` (produce or recycle) and a layout of the arena inventory. 
    *   It defines a formal PDDL-style problem with boolean fluents (e.g., `block_on_shelf`, `toy_assembled`, `arm_free`) to track the state of blocks, toys, and the robot. 
    *   It uses `pyperplan` or `fast-downward` to solve for a sequence of symbolic actions (e.g., `fetch_block()`, `goto_workbench()`).
    *   It translates this symbolic plan into an array of actionable `PlanStep` messages and publishes them to `/planned_task`.
*   **TaskExecutorNode (`temp_warehouse_node.py`):** Acts as the dispatcher. It subscribes to `/planned_task` and iterates through the steps.
    *   It uses a combination of string publishers (e.g., `/go_to_shelf_cmd` with an 800-second timeout) for base navigation and Action Clients (`/arm_command` with an 800-second timeout) for arm manipulation.

### 3.2 MoveIt Task Constructor (MTC) Pipeline
Once the `TaskExecutorNode` fires an Action Goal, the `MTCTaskNode` constructs a state machine to move the arm:

1.  **Preparation Phase:** Captures the current joint state, opens the gripper via Joint Interpolation, and safely navigates to the pre-grasp pose using OMPL TRRT.
2.  **Pick Serial Container:** Computes IK (with a 0.15m Z-offset and $\pi/2$ angle delta), temporarily disables workstation collision checking, approaches linearly in Z (-1.0), closes the gripper, and lifts linearly in world Z (1.0).
3.  **Place Serial Container:** Uses TRRT to move near the target slot, computes the place IK based on dynamic constraints, lowers linearly, opens the gripper, re-enables collision checking, retreats upwards, and returns to the `home` configuration.

### 3.3 Dynamic Scene Management & Constraints
A significant challenge in mobile manipulation is that the robot's base is never in the exact same position relative to the workstation twice. Hardcoding collision obstacles into the planning scene results in failures if the base drifts. 

To solve this, `MTCTaskNode` dynamically manages the planning scene for every individual pick-and-place cycle by injecting parametric collision geometry exactly where the station currently sits relative to the robot.

**Generating Station Geometry:**
Based on the `STATION_TEMPLATES` dictionary, the node injects either a bounding box (for workbenches) or a complex composite shelf (for storage arrays). 

```cpp
// mtc_node2.cpp: Parametrically generating shelf obstacles
auto make_post = [&](const std::string& id, double dx, double dy) {
  moveit_msgs::msg::CollisionObject post;
  post.id = id;
  post.header.frame_id = "base_footprint";
  post.primitives.resize(1);
  post.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  post.primitives[0].dimensions = { 0.9, 0.02 }; // Height, Radius
  post.pose = compose_pose(dx, dy, 0.45); // Extrapolated from the live TF
  return post;
};
// Add corner posts and shelf rows
objects.push_back(make_post("station_post_front_right",  0.43, -0.18));
objects.push_back(make_row("station_shelf_row_bottom", 0.15));
```

**Masking Sibling Blocks:**
When picking a specific block, the planner must not collide with other un-picked blocks in the same station. The node iterates over `BLOCK_MODEL_NAMES` for the target station, skips the requested `picked_slot`, and registers the remaining sibling blocks as `CollisionObject` obstacles in the scene using their live TFs.

```cpp
// mtc_node2.cpp: Registering sibling blocks to prevent collisions
moveit_msgs::msg::CollisionObject obj;
obj.id = "sibling_" + gz_block_name;
obj.header.frame_id = "base_footprint";
obj.primitives.resize(1);
obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
obj.primitives[0].dimensions = { BLOCK_DIM_X, BLOCK_DIM_Y, BLOCK_DIM_Z };
obj.pose = p; // Derived from sibling block's live TF
objects.push_back(obj);
```

### 3.4 Kinematic Transform Hierarchy
The system isolates the robot's local planning scene from Gazebo's absolute coordinate frame to ensure the arm can reliably interact with workstations regardless of base drift. This is achieved in a two-stage hierarchical transform process:

**Stage 1: Absolute to Robot-Relative (Gazebo Relay)**
The `gazebo_station_relay.py` node intercepts absolute world poses directly from Ignition via `/world/empty/pose/info`. Given the robot's world pose $T_{W}^{R}$ and a station's world pose $T_{W}^{S}$, the relay calculates the station's pose strictly relative to the robot's base $T_{R}^{S}$ using the inverse transform:
$$
T_{R}^{S} = (T_{W}^{R})^{-1} \cdot T_{W}^{S}
$$

**Stage 2: Station to Target Slot (MTC Planner)**
Inside `mtc_node2.cpp`, the MoveIt planner queries the TF tree for this `_live` frame. It then composes it with a predefined, local spatial offset corresponding to the specific item slot $T_{S}^{slot}$. This generates the final, robot-relative coordinate $T_{R}^{slot}$ for the place pose:
$$
T_{R}^{slot} = T_{R}^{S} \cdot T_{S}^{slot}
$$

---

## 4. Software/Hardware Interface

The planner communicates primarily through ROS 2 Action servers and TF trees, segregating high-level task requests from low-level joint interpolation.

### 4.1 System Interfaces

* **Action Server**: `arm_command` (Type: `sml_messages/action/ArmCommand`).
* **Task Dispatching**: The executor manages state by listening to `/go_to_shelf_result` and parsing sequences from `/planned_task`.

### Code Snippet: MTC Action Server Initialization
```cpp
// mtc_node2.cpp: Setup of the Action Server for Pick and Place routines
action_server_ = rclcpp_action::create_server<ArmCommand>(
    node_,
    "arm_command",
    std::bind(&MTCTaskNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MTCTaskNode::handleCancel, this, std::placeholders::_1),
    std::bind(&MTCTaskNode::handleAccepted, this, std::placeholders::_1));
```

---

## 5. Dynamic Scene Integration: Gazebo Station Relay

The MoveIt planning scene requires all transforms to trace back to the robot's base frame (`base_footprint`). However, Gazebo Ignition's `/world/empty/pose/info` publisher outputs absolute poses relative to the global simulation origin. The `gazebo_station_relay.py` node mathematically bridges this gap.

### 5.1 Core Transform Mathematics

To compute relative poses, the relay relies on homogeneous 4x4 transformation matrices. It continuously tracks the robot model (`mecanum_arm_robot`) and an array of `STATION_MODELS` and `BLOCK_MODELS`.

For every tick:
1.  **Invert Robot Pose:** The script calculates the inverse matrix of the robot's absolute world pose (`robot_world_inv = np.linalg.inv(self.latest[ROBOT_MODEL_NAME])`).
2.  **Multiply:** It computes the relative matrix via dot product with the target model's absolute pose (`relative = robot_world_inv @ self.latest[gz_model_name]`).
3.  **Decompose:** It extracts the spatial translation (`t = relative[:3, 3]`) and converts the 3x3 rotation matrix block back into a ROS quaternion.

```python
# gazebo_station_relay.py: Executing the matrix multiplication for relative pose
relative = robot_world_inv @ self.latest[gz_model_name]
t = relative[:3, 3]
q = quaternion_from_matrix(relative)

tf_msg = TransformStamped()
tf_msg.header.frame_id = 'base_footprint'
tf_msg.child_frame_id = f'{friendly_name}_live'
# ... (populate translation and rotation)
self.broadcaster.sendTransform(tf_msg)
```

### 5.2 Robust Quaternion Extraction
Converting a 3x3 rotation matrix into a quaternion computationally is notoriously unstable near $\pm 180^{\circ}$ rotations, often resulting in singularities (divide-by-zero) when using standard trace methods. 

To prevent the TF tree from collapsing during simulation, the relay utilizes the **Bar-Itzhack / Shepperd robust eigenvalue method**. This algorithm constructs a symmetric $4 \times 4$ K-matrix from the rotation matrix components and extracts the quaternion as the eigenvector corresponding to the maximum eigenvalue:

```python
# gazebo_station_relay.py: Robust eigenvalue quaternion extraction
K = np.array([
    [m00 - m11 - m22, 0.0,             0.0,             0.0],
    [m01 + m10,       m11 - m00 - m22, 0.0,             0.0],
    [m02 + m20,       m12 + m21,       m22 - m00 - m11, 0.0],
    [m21 - m12,       m02 - m20,       m10 - m01,       m00 + m11 + m22],
]) / 3.0

eigvals, eigvecs = np.linalg.eigh(K)
q = eigvecs[[3, 0, 1, 2], np.argmax(eigvals)]  # [w, x, y, z]
```
By broadcasting these transforms continuously at 10 Hz with the suffix `_live` (e.g., `s_top_live`, `wb_top_live`), the MTC node is guaranteed an accurate, real-time spatial mapping of the simulation environment relative to its base.

---

## 6. Future Work / Known Limitations

While the current architecture successfully unifies the 5 DOF arm and the mecanum base into a mobile manipulator, there are distinct execution constraints and brittleness in the spatial tracking that must be addressed in subsequent revisions.

### 6.1 Action Server Concurrency Lock
**The Limitation:** The `MTCTaskNode` manages task execution using a strictly atomic boolean flag (`busy_.load()`). When an `/arm_command` goal is accepted, `busy_` is set to `true`. If the `TaskExecutorNode` attempts to submit a secondary goal before the current one finishes, the action server outright rejects it rather than queuing it.

```cpp
// mtc_node2.cpp: Atomic concurrency lock
if (busy_.load()) {
  RCLCPP_WARN(LOGGER, "Rejecting goal for '%s' -- a pick-and-place cycle is already running.", goal->object_id.c_str());
  return rclcpp_action::GoalResponse::REJECT;
}
```
**Future Fix:** Implement a robust Goal Queue mechanism utilizing standard `rclcpp_action` preemption policies to allow consecutive task buffering or mid-trajectory aborts.

### 6.2 Hardcoded Spatial Constraints
**The Limitation:** While the planner tracks dynamic workstation locations, the *relative slots within those workstations* (and on the robot's base) are heavily hardcoded into `temp_warehouse_node.py` as explicit array tuples:
```python
# temp_warehouse_node.py: Fixed coordinate mappings
SHELF_SLOTS = [(-0.118, 0.41, 0.00145), (0.50, -0.40, 0.09), (0.60, -0.48, 0.09)]
BASE_SLOTS = [(-0.11, -0.090, 0.19), (-0.11, 0.005, 0.19), (-0.11, 0.100, 0.19)]
```
If the physical dimensions of the storage shelves or the robot's onboard tray change by even a few centimeters, these static indices will result in grasping thin air or colliding with the chassis.
**Future Fix:** Derive `SHELF_SLOTS` and `BASE_SLOTS` parametrically by querying the URDF bounding boxes or implementing dynamic slot calculation based on object dimensions.

### 6.3 Live TF Dependency & Fallback Brittleness
**The Limitation:** The MTC pipeline relies explicitly on the `gazebo_station_relay` node. If this relay crashes or if a TF lookup fails, `mtc_node2.cpp` falls back to static offsets defined in `STATION_TEMPLATES`. If the mecanum base is misaligned when this fallback triggers, the static coordinates will not match reality, directly leading to collisions.
**Future Fix:** Remove the static fallback. Instead, force a safety halt and trigger the `align.py` node to re-center the base precisely at the workstation before attempting to re-plan.

### 6.4 Static Grasp Generation
**The Limitation:** The `GenerateGraspPose` stage in the MTC pipeline uses a statically defined pitch approach (an angle delta of $\pi/2$ mapped to `Eigen::Vector3d::UnitY()`) and a hardcoded Z-axis offset of 0.15 meters. This forces the gripper to approach all blocks top-down, which is insufficient for complex geometries or partially obscured objects.
**Future Fix:** Utilize the `pcl_perception` package to process depth cloud data, extract surface normals via Point Cloud Library (PCL), and dynamically generate continuous grasp approach vectors specific to the orientation of the detected object.
