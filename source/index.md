# RoboManipal — RoboCup Smart Manufacturing League (SML) 2026

```{admonition} Autonomous Mobile Manipulation Platform
:class: tip
Welcome to the central technical documentation portal for **RoboManipal's** autonomous mobile manipulation platform (**KARMA**), engineered for the **RoboCup Smart Manufacturing League (SML) 2026** competition.
```

This portal provides comprehensive engineering documentation across the platform's mechanical architecture, embedded electronics, perception stack, and mission planning systems.

---

## Core Systems

::::{grid} 1 1 2 2
:gutter: 3

:::{grid-item-card} Mobile Base
:link: mobile_base/index
:link-type: doc

**Omnidirectional Mobility & Power**

Chassis mechanics, dead-wheel odometry suspension, power distribution PCB, motor control firmware, and navigation.
:::

:::{grid-item-card} Manipulator
:link: manipulator/index
:link-type: doc

**5-DOF Articulated Arm & End-Effector**

Arm kinematics, cycloidal gearboxes, carbon fiber composite links, joint braking, distributed PCBs, and MoveIt 2.
:::

:::{grid-item-card} Perception
:link: perception/index
:link-type: doc

**3D Vision & Pose Estimation**

RGB-D point cloud processing, YOLO oriented bounding box tracking, RANSAC surface filtering, and 6-DOF grasp pose estimation.
:::

:::{grid-item-card} Autonomy
:link: autonomy/index
:link-type: doc

**Mission Planning & Coordination**

Referee box protocol, automated planning for produce and recycle orders, station management, and action execution.
:::

::::

---

## Platform Technical Specifications

| System Domain | Component / Parameter | Specification / Technology |
| :--- | :--- | :--- |
| **Locomotion** | Drive Kinematics | 4-wheel independent Mecanum omnidirectional drive |
| **Localization** | Planar Tracking | 3-wheel ground-sprung dead-wheel pods + BNO085 IMU + Optical Flow |
| **Manipulator** | Degrees of Freedom | 5-DOF (Yaw-Pitch-Pitch-Pitch-Roll) + Conformal Parallel Gripper |
| **Actuation** | Joint Actuators | NEMA23 Steppers, BLDC motors with cycloidal gearboxes, Dynamixel servos |
| **Primary Compute** | Companion Computer | NVIDIA Jetson AGX Orin (Ubuntu 22.04 LTS, ROS 2 Humble) |
| **Real-Time Control** | Embedded Microcontroller | Dual STM32F411CEU6 (ARM Cortex-M4F @ 96 MHz) running FreeRTOS |
| **Power System** | Supply Bus | 24.0 V Nominal battery pack with selective actuation kill switches |
| **Safety Architecture** | Hardware Watchdog | 150 ms communication deadman cutoff, emergency stop gating |

---

## System Architecture

```{mermaid}
flowchart TD
    subgraph Ref_Layer ["Referee System"]
        Ref["Referee Order Server"] -->|Task Orders| GP["Global Task Planner: UP"]
    end

    subgraph High_Level ["NVIDIA Jetson AGX Orin (ROS 2 Humble)"]
        GP -->|Planned Tasks| Exec["Task Executor"]
        Exec -->|Perception Request| PCL["PCL 3D Perception Pipeline"]
        Exec -->|Arm Commands| MTC["MoveIt 2 Manipulation Planner"]
        Exec -->|Velocity Commands| NAV["Nav2 Mobile Base Navigation"]
    end

    subgraph Low_Level ["Real-Time Embedded Firmware (STM32F411)"]
        MTC -.->|Joint Targets| ArmPCB["Joint Controller PCBs"]
        NAV -.->|Twist Velocity| FreeRTOS["Base Motion Controller"]
        FreeRTOS -->|20 kHz PWM| Motors["Drive Motors and Encoders"]
        Sensors["3x Deadwheels and BNO085 IMU"] -->|50 Hz Telemetry| FreeRTOS
        FreeRTOS -->|Odom and IMU Telemetry| NAV
    end
```

---

```{toctree}
:maxdepth: 2
:caption: Robot Systems
:hidden:

mobile_base/index
manipulator/index
perception/index
autonomy/index
```
