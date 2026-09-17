# Navigation Stack

The autonomous navigation stack provides closed-loop localization, global and local trajectory planning, obstacle avoidance, and sub-centimeter workstation alignment for RoboManipal's RoboCup Smart Manufacturing League (SML) 2026 mobile base.

The robot combines a 4-wheel independent Mecanum drive with a 3-wheel sprung dead-wheel tracking system and a 2D LiDAR. Coarse navigation across the arena is driven by Nav2 and SLAM Toolbox, while precision docking at assembly shelves is executed via real-time laser line extraction.

---

## Demonstration: Autonomous Navigation & SLAM

The video below demonstrates the autonomous mobile base executing real-time SLAM mapping and closed-loop Nav2 trajectory following across the competition arena.

```{raw} html
<div style="text-align: center; margin: 1.5rem 0;">
  <video controls autoplay loop muted style="max-width: 100%; width: 720px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15);">
    <source src="../../_static/Nav2AndSlam.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
  <p style="font-size: 0.9rem; color: #666; margin-top: 0.5rem; font-style: italic;">
    Figure: Nav2 trajectory execution and SLAM mapping in action (4x speed).
  </p>
</div>
```

---

## Navigation Architecture

The navigation pipeline spans embedded microcontroller firmware, ROS 2 device drivers, state estimators, path planners, and specialized alignment controllers:

```{mermaid}
flowchart TD
    subgraph Sensing ["Embedded Sensing Layer"]
        DW["3x Dead-Wheel Encoders"] -->|Serial 115200| SB["serial_bridge Node"]
        BNO["BNO085 IMU"] -->|Serial 115200| SB
        LIDAR["2D LiDAR: /scan"]
    end

    subgraph State_Est ["Odometry and State Estimation"]
        SB -->|/wheel_tick| BRAIN["smart_brain Node"]
        BRAIN -->|Raw Kinematics| EKF["robot_localization EKF"]
        SB -->|/imu/data_raw| EKF
        EKF -->|odom to base_link| TF["TF2 Transform Tree"]
    end

    subgraph Mapping_Loc ["Mapping and Global Localization"]
        LIDAR --> SLAM["slam_toolbox Mapping Mode"]
        SLAM -->|Occupancy Grid| MAP["Static Map Server"]
        MAP --> AMCL["AMCL OmniMotionModel"]
        LIDAR --> AMCL
        AMCL -->|map to odom| TF
    end

    subgraph Motion_Plan ["Nav2 Autonomous Navigation"]
        MAP --> SMAC["SmacPlanner2D Global Planner"]
        LIDAR --> COST["Global and Local Costmaps"]
        SMAC --> MPPI["MPPI Controller FollowPath"]
        COST --> MPPI
        MPPI --> VEL["velocity_smoother: /cmd_vel"]
    end

    subgraph Docking ["Workstation Precision Docking"]
        LIDAR --> LLE["laser_line_extraction"]
        LLE -->|line_segments| ALIGN["align.py Shelf Alignment"]
        ALIGN -->|NavigateToPose| MPPI
    end
```

---

## Navigation Modules

::::{grid} 1 1 3 3
:gutter: 3

:::{grid-item-card} Dead-Wheel Odometry
:link: odometry
:link-type: doc

**Encoder Kinematics & Serial Bridge**

STM32 UART framing, `smart_brain` kinematic integration, wheel tick unpacking, and `odom → base_link` TF broadcasting.
:::

:::{grid-item-card} SLAM & Nav2 Stack
:link: slam_localization
:link-type: doc

**Occupancy Mapping & Path Planning**

SLAM Toolbox mapping, AMCL omnidirectional localization, SmacPlanner2D, MPPI controller with active critics, and layered costmaps.
:::

:::{grid-item-card} Workstation Alignment
:link: shelf_alignment
:link-type: doc

**LiDAR Line Extraction & Docking**

Sub-centimeter pose correction at workstation shelves via split-and-merge line segmentation, temporal filtering, and convergence loops.
:::

::::

---

```{toctree}
:maxdepth: 1
:hidden:

odometry
slam_localization
shelf_alignment
```
