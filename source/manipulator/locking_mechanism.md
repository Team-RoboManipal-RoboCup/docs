# Joint Braking Mechanism

## 1. Overview & Objectives

This document defines the Design for Manufacturing (DFM) and integration architecture for a failsafe braking mechanism on the 5-DOF robotic arm. The system employs an axial solenoid pin-slot architecture designed to hold a static gravitational load without relying on continuous 24V actuation.

### 1.1 Key Design Objectives

- **Torque Rating:** Sustain 3-4 Nm of static holding torque.
- **Mass Constraint:** Total assembly weight strictly < 1.5 kg to minimize overhung mass on lower joints.
- **Power State:** 0 W power consumption during competition kill-switch triggering (failsafe engagement).
- **Actuation Vector:** Axial pin engagement (parallel to the motor shaft) to eliminate cantilever snag hazards inherent to radial designs.

## 2. Technical Specifications & Bill of Materials (BOM)

| Component | Specification / Material | Qty | Function |
|---|---|---|---|
| Locking Disc (Rotor) | Aluminum 6061, 85 mm OD | 1 | Laser-cut with 8 kidney-bean slots to catch the pin. |
| Actuator | 24V Push-Pull Solenoid (JF-1253B) | 1 | Actuates locking pin; relies on an internal return spring for the failsafe drop. |
| Locking Pin | 8.0 mm Steel Rod, 65 mm length | 1 | Acts as a deadbolt to lock the disc to the stator frame. Threaded on one end. |
| Stator Bracket | Laser-cut L-Brackets | 2 | Stacked face-to-face to guide the locking pin. |
| Clevis Fork | PLA/PETG (40% Infill) | 1 | Connects the solenoid plunger to the locking pin via a dual-nut clamping method. |
| Cross-Pin | 3.0 mm Solid Steel Pin | 1 | Connects the plunger to the clevis through a vertical slot. |

```{figure} assets/joint3_brake_assembly.png
:alt: failsafe brake assembly
:width: 100%

Axial solenoid pin-slot brake assembly, showing the locking disc, stator brackets, clevis fork, and solenoid.
```

## 3. Design & Architecture

### 3.1 Kinematics and Shear Loading

The mechanism leverages a large 85 mm disc diameter to minimize shear force on the internal solenoid components. The shear force ($F_{shear}$) exerted on the stator block is inversely proportional to the radius of the locking disc ($R_{disc}$):

$$
F_{shear} = \frac{\tau_{joint}}{R_{disc}}
$$

For a 3-4 Nm torque requirement at an effective radius of ~35 mm (within the 85 mm disc), the shear force drops to approximately 57.14 N, maintaining a high safety factor for the 8.0 mm steel pin.

### 3.2 Locking Disc & Stator Architecture

**Kidney-Bean Slots:** The locking disc is laser-cut with exactly 8 kidney-bean slots. This profile acts as a funnel, allowing the pin to drop smoothly without violent shear impacts if the arm is moving dynamically when power is severed.

**Stacked Stator Brackets:** The stator is formed by two laser-cut L-brackets stacked directly behind one another (face-to-face). This simplifies fabrication while providing a combined material thickness sufficient to resist the ~57.14 N shear load.

```{figure} assets/locking_disc_kidney_slots.png
:alt: Locking disc showing 8 kidney-bean slots
:width: 80%

Locking disc geometry with 8 kidney-bean slots and the stacked L-bracket stator.
```

### 3.3 Linkage Assembly & Motor Interface

The solenoid plunger is linked to the 8.0 mm steel locking pin via a custom 3D-printed clevis fork. The clevis decouples linear plunger motion from radial play in the pin. Dual nuts threaded onto the locking pin allow axial adjustment, ensuring the pin completely disengages from the disc slots when the solenoid is energized.

---

## 4. Software/Hardware Interface

The solenoid is driven via a low-side MOSFET switch controlled by the arm joint MCU. 

```yaml
# Brake Actuation Configuration
arm_brake_actuator:
  ros__parameters:
    solenoid_gpio_pin: 14
    engage_delay_ms: 120
    disengage_delay_ms: 80
    normally_engaged: true
```

---

## 5. Future Work / Known Limitations

* **Pin Chamfer Optimization:** Adding a conical lead-in chamfer to the tip of the locking pin will further reduce alignment sensitivity during high-speed emergency drop tests.
* **Return Spring Upgrades:** Testing stiffer custom die springs to accelerate the failsafe engagement time to under 50 ms.
