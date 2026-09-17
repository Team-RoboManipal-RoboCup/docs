# Manipulator

The manipulator is a custom 5-DOF robotic arm engineered for dexterous object manipulation, assembly, and pick-and-place tasks in the RoboCup Smart Manufacturing League (SML) competition.

It utilizes a Yaw-Pitch-Pitch-Pitch-Roll kinematic configuration with lightweight carbon fiber composite links, high-reduction cycloidal gearboxes, a fail-safe joint braking mechanism, a parallel jaw gripper, distributed joint controller PCBs, and a ROS 2 MoveIt 2 motion planning pipeline. In addition, an AgileX Piper 6-DOF arm serves as a workbench testbed for assembling and disassembling task objects.

```{figure} assets/robotic_arm_cad_assembly.png
:name: fig_manipulator_cad
:alt: Full CAD assembly view of the 5-DOF robotic arm
:width: 80%

Full CAD assembly view of the 5-DOF robotic manipulator arm.
```

```{toctree}
:maxdepth: 2
:caption: Manipulator Modules

cycloidal_gearbox
composite_links
locking_mechanism
kinematics_and_dynamics
gripper
joint_pcb
motors_and_drivers
motion_planning
workbench_arm
```

---

## Technical Specifications & Bill of Materials (BOM)

| Component | Specification | Notes / Purpose |
| :--- | :--- | :--- |
| **Kinematics** | Yaw-Pitch-Pitch-Pitch-Roll (5 DOF) | Dexterous workspace reaching with low mass |
| **2nd & 3rd DOF Actuators** | Turbro D3548 EVO 300kV BLDC | Primary lifting torque for base and shoulder pitch joints |
| **2nd & 3rd DOF Gearboxes** | Custom In-House Cycloidal (20:1) | Integrated directly into the housing of the link |
| **4th DOF Actuator** | Dynamixel AX-18A | Wrist pitch actuation |
| **5th DOF Actuator** | Dynamixel AX-12A | End-effector roll actuation |
| **2nd to 3rd DOF Link** | Carbon Fiber Composite Tubes | Lightweight sandwich structure for bending stiffness |
| **Joint Bearings** | Deep Groove Ball Bearings | Distributes payload and radial stress away from motor shafts |
| **Mounting Hardware** | Knurled Brass Heat Press Inserts | Repeatable, non-destructive modular assembly |
| **Failsafe Brake** | Push-Pull Solenoid Pin-Slot | 0W holding brake sustaining 3–4 Nm torque |
| **End-Effector** | Rack-and-Pinion Gripper | TPU-lined jaws for conforming grip |
