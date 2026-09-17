# Joint Actuators & Motor Drivers

```{admonition} Module Specification
:class: note
Joint Actuation & Motor Drives — RoboManipal | RoboCup SML 2026
```

---

## Overview & Objectives

The 6-DOF robotic arm uses a **heterogeneous actuation strategy** — each joint's motor type, KV rating, and gearbox ratio is independently selected to match the inertia, torque, and speed envelope of that specific DOF rather than applying a single motor family across all joints.

**Actuation summary by DOF:**

| DOF | Joint Role | Motor Type | Driver | Gearbox |
|-----|-----------|------------|--------|---------|
| 1 | Shoulder Rotation (base) | NEMA23 Stepper | DM860H | Worm 15:1 |
| 2 | Shoulder Elevation | NEMA23 Stepper | DM860H | Worm 15:1 |
| 3 | Elbow | Turbo D3548 EVO 300 KV (BLDC) | NMotion NDrive Z1 | S44 Planetary 85:1 |
| 4 | Wrist Pitch | Turbo D3536 EVO 910 KV (BLDC) | NMotion NDrive Z1 | S38 Planetary 23:1 |
| 5 | Wrist Roll | Turbo D3536 EVO 910 KV (BLDC) | NMotion NDrive Z1 | S38 Planetary 23:1 |
| 6 + Gripper | End-Effector + Grasp | Dynamixel AX-12A (Smart Servo) | Integrated (TTL) | Integrated planetary |

**Closed-loop feedback is implemented on every motor** — steppers via encoder-augmented step-loss detection (DM860H encoder port), BLDC joints via the NDrive Z1's onboard 14-bit MA732 absolute magnetic encoder + optional external incremental encoder, and the AX-12A via its integrated position sensor.

**Design objective:** Achieve at least 10 N·m at the elbow (DOF 3) and at least 3 N·m at the wrist (DOF 4/5) while maintaining a total arm weight budget that does not exceed the robot's payload margin for SML task manipulation.

---

## Technical Specifications & Bill of Materials (BOM)

### DOF 1 & 2 — NEMA23 Stepper + Worm Gearbox

#### Motor: NEMA23 Stepper (RMCS-2061 / Robokits)

| Parameter | Value |
|-----------|-------|
| Frame | NEMA23 (57 mm flange) |
| Step angle | 1.8° (200 steps/rev) |
| Configuration | 4-wire bipolar |
| Rated voltage | 3.2 V |
| Phase current | 4 A |
| Resistance / phase | 0.35 Ω |
| Inductance / phase | 1.8 mH |
| Rotor inertia | 480 g·cm² |
| Insulation class | Class B |
| Motor body length | 102 mm |
| Shaft diameter | 8 mm |
| Shaft length | 20 mm |
| Holding torque | **220 kg·cm** (post-gearbox) |

#### Gearbox: Worm Reducer, 15:1

| Parameter | Value |
|-----------|-------|
| Type | Worm gear reducer |
| Ratio | 1:15 |
| Rated output torque | 220 kg·cm |
| Installation | Horizontal |
| Output shaft diameter (Dm) | 14 mm |
| Gearbox weight | 1.2 kg |
| Combined (motor + gearbox) | 2.6 kg |
| Product reference | [RMCS-2061 — Robokits India](https://robokits.co.in/motors/stepper-motor/stepper-motor-with-gearbox/nema23-stepper-motor-with-worm-gearbox-220kgcm) |

#### Driver: DM860H Microstepping Stepper Driver

| Parameter | Value |
|-----------|-------|
| Supply voltage | 18–80 V AC / 24–110 V DC |
| Output current (peak) | 2.4–7.2 A (DIP-configurable) |
| Microstep resolution | 400–25,600 ppr (DIP-configurable) |
| Max pulse frequency | 200 kHz |
| Signal input | Single-ended, 5–24 V Pulse/Direction |
| Dimensions | 150 × 109 × 53 mm |
| Weight | ~483 g |
| Protection | OV, UV, OC, thermal |

**DIP switch current configuration (SW1–SW3) for NEMA23 at 4 A:**

```text
SW1=ON  SW2=OFF  SW3=ON  → 4.20 A peak output
SW4=OFF → idle current = 50% (reduces heat at rest)
```

---

### DOF 3 — Turbo D3548 EVO 300 KV + S44 Planetary 85:1 + NDrive Z1

#### Motor: Turbo D3548 EVO 300 KV (BLDC Outrunner)

| Parameter | Value |
|-----------|-------|
| Stator dimensions | Ø35.3 mm × 48 mm |
| Winding | 12N14P (12 stator arms, 14 poles) |
| KV rating | 300 KV (custom-wound low-KV variant) |
| Shaft diameter | 5.0 mm |
| Shaft length | 65.5 mm |
| Weight | ~170 g (incl. wires) |
| Bell material | CNC 6061-T6 aluminum |
| Bearings | NSK/NMB precision |
| Mounting pattern | 19 × 25 mm (M3 × 4) |

```{admonition} Why 300 KV for DOF 3?
:class: important
The elbow joint (DOF 3) operates against the highest reflected inertia of the distal arm
links. A **lower KV (300 KV)** motor produces more torque per amp at the same voltage
compared to the 910 KV variants used in DOF 4/5. Pairing with the **85:1 planetary gearbox**
yields a theoretical output torque of approximately 12–14 N·m at the elbow shaft (assuming
~0.18 N·m motor stall torque at rated current, 48 V bus), which comfortably covers the
estimated 8–10 N·m load case for pick-and-place tasks.
```

**Effective output torque estimate:**

```
τ_output = τ_motor × GR × η_gearbox
         ≈ 0.18 N·m × 85 × 0.85
         ≈ 13.0 N·m  (at stall, 48 V, η_planetary ≈ 85%)
```

#### Gearbox: Ingenium Drives S44 Planetary, 85:1

| Parameter | Value |
|-----------|-------|
| Type | Epicyclic (planetary) |
| Ratio | 85:1 |
| Gear material | Steel alloy |
| Output shaft diameter | 12 mm |
| Output shaft material | Hardened H.T. steel alloy |
| Breaking torque | ~98 N·m |
| Assembly weight | ~900 g–1 kg |
| Product page | [S44 Planetary Gearbox 85:1 — Ingenium Drives](https://ingeniumdrives.in/product/s44-planetary-gearbox-85-1/) |

```{admonition} Why Planetary over Worm for DOF 3?
:class: note
Unlike the worm gearboxes used in DOF 1/2 (which are deliberately **self-locking** to hold
shoulder pose under gravity), DOF 3 requires **bi-directional back-driveability** for
compliant interaction and fast reversals. Planetary gearboxes have significantly lower
gear-mesh losses (~85–92% efficiency) vs. worm gears (~40–70%), enabling regenerative
braking and direct torque control via the NDrive Z1's FOC algorithm. The self-locking
property of a worm at this stage would prevent gravity-compliant behaviour needed for
SML task manipulation.
```

---

### DOF 4 & 5 — Turbo D3536 EVO 910 KV + S38 Planetary 23:1 + NDrive Z1

#### Motor: Turbo D3536 EVO 910 KV (BLDC Outrunner)

| Parameter | Value |
|-----------|-------|
| Stator dimensions | Ø35.3 mm × 36 mm |
| Winding | 12N14P |
| KV rating | 910 KV |
| Max power | ~347.7 W |
| Max current | 21.45–25.5 A |
| Weight | ~105.8 g (incl. wires) |
| Shaft diameter | 5.0 mm |
| Shaft length | 53.5 mm |
| Mounting pattern | 19 × 25 mm (M3) |
| Compatible supply | 3S–4S LiPo (≤24 V) |

```{admonition} Why 910 KV for DOF 4/5?
:class: important
The wrist joints (DOF 4/5) carry far less reflected inertia than the elbow but must execute
**faster, lighter movements** during orientation adjustments in SML tasks. The 910 KV
variant at 24 V spins at approximately 21,840 RPM no-load; after the **23:1 reduction**
this yields ~950 RPM at the wrist output shaft — sufficient angular velocity for rapid
reorientation. At this ratio, stall torque is approximately **3–4 N·m**, adequate for
grasping and tool-holding workloads at the wrist. Using the **lighter D3536** (105.8 g vs.
170 g D3548) also reduces the arm's distal mass, directly lowering the torque demand on
DOF 3 (elbow).
```

**Effective output torque estimate:**

```
τ_output = τ_motor × GR × η_gearbox
         ≈ 0.17 N·m × 23 × 0.88
         ≈ 3.44 N·m  (at stall, 24 V, η_planetary ≈ 88%)
```

#### Gearbox: Ingenium Drives S38 Planetary, 23:1

| Parameter | Value |
|-----------|-------|
| Type | Epicyclic (planetary) |
| Ratio | 23:1 |
| Gear material | Hardened steel alloy |
| Output shaft diameter | 10 mm |
| Output shaft length | 35 mm |
| Max continuous torque | ~68–72 kg·cm |
| Assembly weight | ~320–400 g |
| Product page | [S38 Planetary Gearbox 23:1 — Ingenium Drives](https://ingeniumdrives.in/product/s38-planetary-gearbox-23-1/) |

---

### DOF 3, 4 & 5 — Driver: NMotion NDrive Z1

The NDrive Z1 is an Indian-designed Field-Oriented Control (FOC) BLDC motor driver produced by NMotion ([nmotion.in](https://www.nmotion.in)). It replaces conventional ESC/PWM BLDC drivers with a closed-loop, torque-controlled architecture communicating over CAN 2.0B — a significant capability upgrade for precision robotic arm control.

| Parameter | Value |
|-----------|-------|
| Control algorithm | FOC (Field-Oriented Control) |
| Input voltage | 12–48 V DC (handles peak up to 56 V) |
| Continuous current | 20 A |
| Peak current | 55 A (for 2 s) |
| Board dimensions | 52 × 52 mm |
| Communication | CAN 2.0B, up to 1 Mbps |
| Onboard encoder | 14-bit absolute magnetic (MA732) |
| External encoder inputs | ABZ incremental (quadrature) + SPI absolute |
| Dual-encoder mode | Yes (any two of the three encoder sources) |
| Control modes | Position, Velocity, Torque, Trapezoidal Trajectory, Ramped Velocity |
| Motor thermistor input | Yes (external NTC) |
| Brake resistor support | Yes (dumps regenerative currents if supply cannot sink) |
| CAN termination | Onboard DIP-switch selectable 120 Ω |
| SDK | Python / C++ via NMotion Transport Library; NMotion CLI |
| Firmware status | Production-Ready (2026) |
| Product docs | [docs.nmotion.in/hardware/motor-drivers/ndrive-z1](https://docs.nmotion.in/hardware/motor-drivers/ndrive-z1) |

```{admonition} Why NDrive Z1 over a Generic ESC?
:class: important
Standard BLDC ESCs (BLHeli32, AM32) operate in **open-loop RPM mode** via PWM with no direct
torque or position control. The NDrive Z1 implements **true FOC**, enabling:

- **Closed-loop torque control** at every BLDC joint (DOF 3/4/5), critical for compliant
  force-controlled grasping in SML manipulation tasks.
- **CAN bus daisy-chain topology**: all three NDrive Z1 units share a single 2-wire bus to
  the main compute board, eliminating per-driver signal cable runs.
- **Dual absolute encoder support** enables output-side feedback (encoder mounted after the
  gearbox output shaft) for full closed-loop position control that accounts for gearbox backlash.
- **Multiple control modes** selectable at runtime: the arm can switch from velocity (transit)
  to torque (contact/grasp) to trapezoidal trajectory (motion-planned moves) per task phase.
```

---

### DOF 6 & Gripper — Dynamixel AX-12A Smart Serial Servo

| Parameter | Value |
|-----------|-------|
| Part number | Dynamixel AX-12A |
| Operating voltage | 9.0–12.0 V (recommended 11.1 V / 3S LiPo) |
| Stall torque | 1.5 N·m (at 12 V, 1.5 A) |
| No-load speed | 59 RPM (at 12 V) |
| Angular resolution | 0.29° |
| Operating angle | 0°–300° (position mode) / continuous (wheel mode) |
| Weight | 54.6 g |
| Dimensions | 32 × 50 × 40 mm |
| Operating temperature | −5°C to 70°C |
| Communication protocol | Dynamixel Protocol 1.0 (half-duplex TTL serial) |
| Baud rate | 7,343 bps – 1 Mbps (register-configurable) |
| Physical bus | TTL, multi-drop (daisy-chainable) |
| Feedback | Position, temperature, load, input voltage |
| Half-duplex interface | 74HC126 tri-state buffer or U2D2 / OpenRB-150 adapter |

```{admonition} Why AX-12A for DOF 6 and Gripper?
:class: note
DOF 6 (wrist rotation) and the gripper require **sub-degree position repeatability** and
**integrated force sensing via load feedback** — capabilities the AX-12A provides out of
the box at low cost and weight (54.6 g). Its daisy-chain TTL bus allows both actuators to
share a single UART port on the compute board. The 1.5 N·m stall torque is sufficient for
the gripper jaw mechanism with the mechanical advantage of the jaw linkage (estimated 4–6 N
fingertip force at 30 mm jaw radius). Its **integrated gearbox, motor, and sensor** removes
the need for a separate driver board, reducing BOM count and wiring complexity for this
precision, low-power stage.
```

---

## Design & Architecture

### Joint Selection Rationale Summary

```{mermaid}
flowchart LR
    A["DOF 1 and 2: Shoulder<br/>NEMA23 with Worm 15:1<br/>DM860H Closed-Loop<br/>Self-locking, High Torque"] --> B["DOF 3: Elbow<br/>D3548 EVO 300KV<br/>Planetary 85:1<br/>NDrive Z1 FOC: 13 Nm"]
    B --> C["DOF 4 and 5: Wrist<br/>D3536 EVO 910KV<br/>Planetary 23:1<br/>NDrive Z1 FOC: 3.4 Nm"]
    C --> D["DOF 6 and Gripper<br/>Dynamixel AX-12A<br/>TTL Smart Servo<br/>Position and Load Feedback"]
```

### Worm Gearbox Self-Locking at DOF 1 & 2

The worm gearbox (15:1) is selected for DOF 1 and 2 because the **lead angle** of the worm is below the friction angle, making it **mechanically self-locking**:

- The shoulder joints maintain their pose under gravity load **without consuming motor power or holding current**.
- In case of E-stop or power loss, the arm **freezes in position** rather than dropping.
- The DM860H **idle current reduction (SW4=OFF)** drops phase current to 50% when stationary — the worm self-lock means no torque is sacrificed.

**Self-locking condition:**

```
tan(λ) < μ
where:
  λ = worm lead angle
  μ = coefficient of friction (typically 0.1–0.15 for steel-on-bronze mesh)

For 15:1 single-thread worm:
  λ ≈ 3.8°
  tan(3.8°) ≈ 0.066 < μ  →  self-locking confirmed
```

### Closed-Loop Feedback Architecture (All 6 DOF)

```text
┌──────────────────────────────────────────────────────────────────┐
│  DOF 1/2  (Stepper + DM860H)                                     │
│  STM32 Step pulse → DM860H → Motor phases → Encoder (ABZ)        │
│  Encoder feedback → DM860H closed-loop step correction           │
│  Position error correction at driver level (closed-loop mode)    │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  DOF 3/4/5  (BLDC + NDrive Z1)                                   │
│  ROS 2 setpoint → CAN @ 1 Mbps → NDrive Z1 → FOC current loop   │
│  Feedback: onboard MA732 14-bit abs encoder (motor shaft)        │
│  + optional ABZ encoder on gearbox output shaft                  │
│  FOC update rate: ~20 kHz; position/velocity loop: ~1 kHz        │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│  DOF 6 + Gripper  (Dynamixel AX-12A)                             │
│  UART (1 Mbps, half-duplex TTL) Instruction Packet → AX-12A     │
│  Integrated Hall / potentiometer position sensor                 │
│  Status Packet returns: position, load, temp, voltage → host     │
└──────────────────────────────────────────────────────────────────┘
```

---

## Software / Hardware Interface

### CAN Bus — NDrive Z1 (DOF 3, 4, 5)

All three NDrive Z1 units are daisy-chained on a single CAN 2.0B bus at **1 Mbps**. Each unit is assigned a unique Node ID (1–127).

**CAN frame addressing (NMotion custom protocol):**

```text
CAN ID = (Node_ID << 5) | Command_Type
Payload = 8 bytes  (float32 setpoint + uint16 flags + uint8 mode + uint8 reserved)
```

**ROS 2 interface (via CAN-to-ROS bridge node on Jetson / STM32):**

| ROS 2 Topic | Message Type | Direction | Description |
|-------------|-------------|-----------|-------------|
| `/arm/joint_3/cmd_pos` | `std_msgs/msg/Float64` | Pub | Elbow position setpoint (rad) |
| `/arm/joint_3/cmd_vel` | `std_msgs/msg/Float64` | Pub | Elbow velocity setpoint (rad/s) |
| `/arm/joint_3/cmd_torque` | `std_msgs/msg/Float64` | Pub | Elbow torque setpoint (N·m) |
| `/arm/joint_3/state` | `sensor_msgs/msg/JointState` | Sub | Position, velocity, effort |
| `/arm/joint_4/cmd_pos` | `std_msgs/msg/Float64` | Pub | Wrist pitch position setpoint |
| `/arm/joint_5/cmd_pos` | `std_msgs/msg/Float64` | Pub | Wrist roll position setpoint |
| `/arm/joint_4/state` | `sensor_msgs/msg/JointState` | Sub | Wrist pitch feedback |
| `/arm/joint_5/state` | `sensor_msgs/msg/JointState` | Sub | Wrist roll feedback |
| `/arm/ndrive/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Sub | Driver temp, current, fault flags |

**NDrive Z1 CAN connector pinout:**

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | CAN_H | Twisted pair with CAN_L |
| 2 | CAN_L | Twisted pair with CAN_H |
| 3 | GND | Common ground to power supply |
| 4 | VBUS | 12–48 V power input |

**Termination:** Set the onboard DIP switch to ON on the **last device** in the daisy chain to connect the built-in 120 Ω termination resistor. All intermediate nodes leave the switch OFF.

---

### Step/Direction — DM860H (DOF 1, 2)

| Signal | Electrical Level | Notes |
|--------|-----------------|-------|
| PUL+ / PUL− | 5–24 V differential or single-ended | Step pulse, max 200 kHz |
| DIR+ / DIR− | 5–24 V | Direction; must settle 5 µs before PUL edge |
| ENA+ / ENA− | 5–24 V | Motor enable (active LOW default) |

**STM32 timer configuration for 10 RPM at gearbox output:**

```c
/* 1/16 microstepping: 200 steps/rev × 16 = 3200 ppr */
#define STEPS_PER_REV   3200UL
#define GEAR_RATIO      15UL

/* Target: 10 RPM at gearbox output shaft */
uint32_t output_rpm  = 10;
uint32_t motor_rpm   = output_rpm * GEAR_RATIO;           /* 150 RPM */
uint32_t step_freq   = (motor_rpm * STEPS_PER_REV) / 60; /* 8000 Hz */

/* Configure TIM2 for 8 kHz PWM (50% duty cycle → step pulses) */
__HAL_TIM_SET_AUTORELOAD(&htim2, (SystemCoreClock / step_freq) - 1);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                       (__HAL_TIM_GET_AUTORELOAD(&htim2) / 2));
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
```

---

### TTL Serial — Dynamixel AX-12A (DOF 6 + Gripper)

**Bus configuration:**

| Parameter | Value |
|-----------|-------|
| Protocol | Dynamixel Protocol 1.0 |
| Physical | Half-duplex TTL (single DATA wire, bi-directional) |
| Default baud rate | 1,000,000 bps (register 4, value = 1) |
| Usable baud rates | 7,343 – 1,000,000 bps |
| Bus interface | 74HC126 tri-state buffer or U2D2 / OpenRB-150 adapter |

**Instruction packet structure:**

```
0xFF 0xFF [ID] [LEN] [INSTRUCTION] [PARAM_1] ... [PARAM_N] [CHECKSUM]
CHECKSUM = ~(ID + LEN + INSTRUCTION + Σ PARAM_i) & 0xFF
```

**Example — Set AX-12A ID=1 goal position to 512 (150°, centre):**

```text
TX: 0xFF 0xFF 0x01 0x05 0x03 0x1E 0x00 0x02 0xD4
     |    |    ID  LEN  WRT  REG  LOW  HI   CHK
     HEADER         = 0x03 (Write Data)
                      REG  = 0x1E (Goal Position register)
```

**Baud rate configuration (Python, dynamixel_sdk):**

```python
from dynamixel_sdk import PortHandler, PacketHandler

portHandler   = PortHandler('/dev/ttyUSB0')
packetHandler = PacketHandler(1.0)  # Protocol 1.0

portHandler.openPort()
portHandler.setBaudRate(1000000)  # Default factory baud rate

DXL_ID = 1  # AX-12A node ID

# Write baud rate register (addr=4): value 1 → 1 Mbps
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL_ID, 4, 1
)
if dxl_comm_result != 0:
    raise RuntimeError(packetHandler.getTxRxResult(dxl_comm_result))
```

**ROS 2 interface (dynamixel_sdk_ros2 package):**

| ROS 2 Topic | Message Type | Description |
|-------------|-------------|-------------|
| `/arm/joint_6/cmd_pos` | `std_msgs/msg/Int32` | Wrist AX-12A goal position (0–1023) |
| `/arm/gripper/cmd_pos` | `std_msgs/msg/Int32` | Gripper AX-12A goal position (0–1023) |
| `/arm/dynamixel/state` | `sensor_msgs/msg/JointState` | Position (rad), velocity (rad/s), effort (%) |

---

## Future Work / Known Limitations

- **D3548 EVO 300 KV availability:** The 300 KV variant is a non-standard custom winding not listed in Flash Hobby's standard catalogue (standard KVs: 760, 900, 1150). A validated fallback is the **D3548 EVO 760 KV** with an adjusted planetary ratio (S44 100:1 or 120:1) to match the target output torque. Motor constants (Kt, Kv, phase resistance) must be measured for both variants before the next revision.

- **Worm gearbox thermal drift (DOF 1/2):** Extended near-rated-torque operation generates significant heat in the worm mesh (30–40% losses). An NTC thermistor point should be added to the gearbox housing in the next mechanical revision and monitored via the `/diagnostics` topic.

- **NDrive Z1 output-side encoder not yet wired (DOF 3/4/5):** Currently only the onboard MA732 (motor shaft) encoder is active. Gearbox backlash on the S44 is estimated at 0.5–1.5° and is not currently compensated. Wiring an ABZ quadrature encoder on each gearbox output shaft and enabling dual-encoder mode on the NDrive Z1 will close this error loop.

- **AX-12A torque headroom (DOF 6/Gripper):** The 1.5 N·m stall torque at 12 V is marginal for grasping objects heavier than ~500 g with the current jaw linkage ratio. Upgrading to **Dynamixel XL430-W250-T** (2.5 N·m, Protocol 2.0) is recommended if the payload exceeds this limit or if a more rigid jaw design reduces mechanical advantage.

- **CAN bus single-point-of-failure:** All three NDrive Z1 units share a single CAN bus. A wiring fault at any intermediate node can degrade communication with downstream units. CAN bus redundancy (secondary bus via separate STM32 CAN peripheral) or per-node health monitoring via the `/arm/ndrive/diagnostics` topic is planned for the next integration phase.

- **Stepper closed-loop validation under load:** The DM860H closed-loop encoder correction on DOF 1/2 has not been characterized under the full gravitational torque profile of the arm. Dynamic torque profiles and step-loss event frequency should be logged during integration testing with the full arm assembly.

---

*Documentation prepared for RoboManipal RoboCup SML 2026.*
