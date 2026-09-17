# Joint Controller PCB

The **Arm Joint Control PCB** (`ENCODER_USB`) is a distributed, per-joint embedded acquisition and actuator control node engineered for the RoboManipal RoboCup SML manipulator. Mounting directly on the rear frame of each joint actuator, it localizes closed-loop trajectory tracking, hardware step generation, and magnetic absolute angle sensing, establishing an electrically isolated serial communication link with the central NVIDIA Jetson AGX Orin computing platform.

```{figure} assets/arm_pcb_kicad.jpg
:name: fig_arm_pcb_kicad
:alt: Arm Joint Control PCB 3D KiCad Assembly
:width: 75%
:align: center

Arm Joint Control PCB (`ENCODER_USB`) 3D assembly and layout rendered from KiCad.
```

---

## 1. Overview & Objectives

Centralized robotic arm wiring introduces severe signal degradation, electromagnetic interference (EMI) from motor PWM harmonics, and bulky cable harnesses that restrict multi-axis joint rotation. The Arm Joint Control PCB addresses these challenges through a modular, distributed architecture:

* **Localized Actuation & Sensing**: Interfaces directly to an on-axis diametric magnet via an AMS AS5600 12-bit rotary encoder, eliminating zero-position calibration routines and back-emf measurement inaccuracies.
* **Dual Actuator Paradigm**: Provides pin-compatible physical and electrical breakout headers for external industrial microstepping drives (Leadshine DMA860H, TB6600) as well as daisy-chained Robotis Dynamixel smart servos running half-duplex TTL asynchronous serial.
* **Complete Galvanic Separation**: Integrates an Analog Devices ADuM4160 digital isolator to decouple the host Jetson AGX Orin USB root complex and digital logic ground from actuator flyback spikes, high ground-bounce currents, and external 12V supply noise.
* **Standardized Form Factor**: Circular/square hybrid footprint designed in KiCad to fit within a 52 mm NEMA 17/NEMA 23 actuator profile with mounting holes aligned for direct mechanical integration.

---

## 2. Technical Specifications & Bill of Materials (BOM)

### 2.1 Hardware Specifications

| Parameter                | Specification                                        | Operating Limit / Design Condition                                             |
|:------------------------ |:---------------------------------------------------- |:------------------------------------------------------------------------------ |
| **Main Compute MCU**     | STM32F103C8T6 (ARM Cortex-M3)                        | 72 MHz core frequency, 64 KB Flash, 20 KB SRAM                                 |
| **Encoder IC**           | AMS AS5600                                           | 12-bit on-axis magnetic ($0.0879^\circ$ resolution / 4096 CPR)                 |
| **Encoder Interface**    | $I^2C$ Fast-Mode (400 kHz)                           | Address `0x36` (7-bit unshifted)                                               |
| **Galvanic Isolation**   | ADuM4160BRWZ                                         | $5.0\text{ kV}_{\text{RMS}}$ dielectric rating, USB 2.0 Full-Speed (12 Mbps)   |
| **Logic Level Shifter**  | TI TXS0108EPWR                                       | 8-bit bidirectional auto-direction ($3.3\text{V} \leftrightarrow 5.0\text{V}$) |
| **CAN Bus Transceiver**  | Microchip MCP2551-I/SN                               | Up to 1 Mbps ISO-11898 compliant physical layer                                |
| **Input Supply Rails**   | Dual-source: 12V DC auxiliary / 5V USB               | Terminal block for 12V; USB $V_{BUS}$ for primary isolated domain              |
| **Regulated Rails**      | 5.0V DC (AMS1117-5.0) / 3.3V DC (AMS1117-3.3)        | Linear cascade topology with thermal stitching arrays                          |
| **Stepper Pulse Output** | 5.0V open-drain/push-pull pulses                     | Up to 200 kHz step frequency via hardware timer `TIM2_CH1`                     |
| **Servo Data Interface** | Single-wire half-duplex UART                         | 57.6 kbps to 1.0 Mbps baud rate                                                |
| **PCB Dimensions**       | $50.0\text{ mm} \times 48.0\text{ mm}$, 2-layer FR-4 | 1.6 mm thickness, 1 oz copper ($35\;\mu\text{m}$), ENIG finish                 |

### 2.2 Bill of Materials (BOM)

| Designator         | Part Number / Value      | Package      | Description / Function                                            |
|:------------------ |:------------------------ |:------------ |:----------------------------------------------------------------- |
| **U1**             | STM32F103C8T6            | LQFP-48      | Microcontroller, 72 MHz ARM Cortex-M3                             |
| **U2**             | AS5600-ASOT              | SOIC-8       | 12-bit programmable magnetic rotary position sensor               |
| **U3**             | ADuM4160BRWZ             | SOIC-16W     | Full-Speed USB digital isolator, $5\text{ kV}_{\text{RMS}}$       |
| **U4**             | TXS0108EPWR              | TSSOP-20     | 8-channel bidirectional level shifter with auto direction         |
| **U5**             | MCP2551-I/SN             | SOIC-8       | High-speed CAN transceiver (unpopulated / reserved)               |
| **U6**             | AMS1117-5.0              | SOT-223      | Linear regulator: $12\text{V} \to 5\text{V}$, 1.0A max            |
| **U7**             | AMS1117-3.3              | SOT-223      | Linear regulator: $5\text{V} \to 3.3\text{V}$, 1.0A max           |
| **Y1**             | 8.000 MHz Crystal        | HC-49/SMD    | High-speed external (HSE) clock oscillator for STM32              |
| **Y2**             | 32.768 kHz Crystal       | Cylinder/SMD | Low-speed external (LSE) RTC crystal (optional)                   |
| **C1, C2**         | 20 pF                    | 0603         | Crystal load caps for 8 MHz crystal ($C_L = 12\text{ pF}$)        |
| **C3, C4, C5, C6** | 100 nF (0.1 µF)          | 0603         | Ceramic X7R decoupling capacitors for MCU $V_{DD}$ pins           |
| **C7, C8**         | 10 µF                    | 0805         | Ceramic input/output decoupling for AMS1117 regulators            |
| **C9, C10**        | 100 nF (0.1 µF)          | 0603         | Decoupling for ADuM4160 $V_{DD1}$, $V_{DD2}$                      |
| **C11**            | 1 nF / 2 kV              | 1812         | High-voltage stitching capacitor across isolated ground barrier   |
| **R1, R2**         | 4.7 kΩ                   | 0603         | $I^2C$ pull-up resistors for `SDA` and `SCL` (3.3V domain)        |
| **R3, R4**         | 24 Ω                     | 0603         | USB $D+$ and $D-$ series damping resistors                        |
| **R5**             | 1.5 kΩ                   | 0603         | USB $D+$ pull-up resistor (asserts Full-Speed 12 Mbps connection) |
| **R6, R7, R8**     | 10 kΩ                    | 0603         | Pull-down resistors on Stepper `ENABLE`, `DIR`, and `BOOT0`       |
| **D1**             | Green SMD LED            | 0603         | 12V rail power indicator                                          |
| **D2**             | Yellow SMD LED           | 0603         | 5V rail power indicator                                           |
| **D3**             | Blue SMD LED             | 0603         | Status / Heartbeat indicator connected to MCU `PC13`              |
| **F1**             | 0.5A Polyfuse            | 1206         | PPTC resettable fuse for USB $V_{BUS}$ power line                 |
| **F2**             | 2.0A Polyfuse            | 1812         | PPTC resettable fuse for 12V auxiliary power input                |
| **J1**             | USB Type-B / Type-C      | Through-Hole | Host interface connector to NVIDIA Jetson AGX Orin                |
| **J2**             | 2-pin 5.08mm Terminal    | Through-Hole | 12V DC actuator bus power terminal                                |
| **J3**             | 4-pin JST-XH (2.5mm)     | Through-Hole | Stepper interface: `PUL+`, `DIR+`, `EN+`, `GND`                   |
| **J4**             | 3-pin Molex Spox (2.5mm) | Through-Hole | Dynamixel bus: `DATA`, `VDD`, `GND`                               |
| **J5**             | 4-pin 2.54mm Header      | Through-Hole | Serial Wire Debug (SWD): `3.3V`, `SWDIO`, `SWCLK`, `GND`          |

---

## 3. Design & Architecture

```
+-------------------------------------------------------------------------+
|                               HOST DOMAIN                               |
|  +--------------------+        USB 2.0         +---------------------+  |
|  | NVIDIA Jetson Host | <====================> | ADuM4160 Isolator   |  |
|  | (5V VBUS / GND_USB)|                        | (Primary Side VDD1) |  |
|  +--------------------+                        +---------------------+  |
+-----------------------------------|-------------------------------------+
                                    | 5 kV Isolation Barrier
                                    | (6.0 mm Creepage Slot)
+-----------------------------------|-------------------------------------+
|                               JOINT DOMAIN                              |
|  +---------------------+                       +---------------------+  |
|  | 12V DC Auxiliary    | ----> AMS1117-5.0 --->| AMS1117-3.3         |  |
|  +---------------------+           |           +----------+----------+  |
|             |                      v                      v             |
|             |                 TXS0108E 5V            STM32F103 (3.3V)   |
|             v                      |                      |             |
|       Actuator Power         +-----+-----+           +----+----+        |
|                              |           |           |         |        |
|                         Stepper Pul  Dynamixel TTL  AS5600   Status LED |
+-------------------------------------------------------------------------+
```

### 3.1 Power Regulation & Thermal Calculation

The PCB is supplied through two distinct power inputs:

1. **Host USB $V_{BUS}$ (5.0V)**: Connects strictly to the primary side of the ADuM4160 isolator (`VDD1`). This ensures complete non-reliance on motor supplies to maintain active enumeration on the Jetson OS.
2. **Auxiliary 12V DC**: Supplies external actuators and feeds the onboard linear regulation cascade.

#### Linear Regulator Power Dissipation

The AMS1117-5.0 steps down the $12\text{ V}$ rail to $5\text{ V}$. The total current through this regulator includes:

* MCU 3.3V rail consumption (via AMS1117-3.3): $I_{3.3} \approx 45\text{ mA}$
* Sensor and Level Shifter: $I_{aux} \approx 15\text{ mA}$
* External optocoupler drive currents (TB6600/DMA860H input stage LEDs at 10 mA per channel): $I_{opto} \approx 30\text{ mA}$
* Status LEDs and divider quiescent currents: $I_{LED} \approx 10\text{ mA}$
* Total current $I_{5V} \approx 100\text{ mA} = 0.10\text{ A}$

Power dissipation $P_D$ across the AMS1117-5.0:

$$P_D = (V_{IN} - V_{OUT}) \cdot I_{5V} = (12.0\text{ V} - 5.0\text{ V}) \cdot 0.10\text{ A} = 0.70\text{ W}$$

Using the SOT-223 junction-to-ambient thermal resistance $\theta_{JA} \approx 90^\circ\text{C/W}$ (with standard copper ground pour heatsinking):

$$\Delta T = P_D \cdot \theta_{JA} = 0.70\text{ W} \cdot 90^\circ\text{C/W} = 63.0^\circ\text{C}$$

With an ambient temperature inside the arm joint casing of $T_A = 40^\circ\text{C}$:

$$T_J = T_A + \Delta T = 40^\circ\text{C} + 63.0^\circ\text{C} = 103.0^\circ\text{C}$$

Because $T_J < T_{J,\text{max}}$ ($125^\circ\text{C}$), the thermal budget is safe for continuous operation. However, the PCB includes an array of twelve $0.3\text{ mm}$ thermal vias beneath the exposed tab to conduct heat directly into the bottom ground plane, lowering effective $\theta_{JA}$ to $\approx 65^\circ\text{C/W}$ ($T_J \approx 85.5^\circ\text{C}$).

---

### 3.2 Galvanic USB Isolation & Ground Separation

High-current step transitions and inductive transients produce high-amplitude ground bounce across long returns:

$$V_{\text{bounce}} = L_{\text{cable}} \cdot \frac{di}{dt}$$

For typical arm cable lengths ($L \approx 1.5\,\mu\text{H}$) and switching transients ($di/dt \approx 2\text{ A} / 10\text{ ns} = 2 \times 10^8\text{ A/s}$), ground bounce voltages can easily exceed $50\text{ V}$, which would destroy unbuffered USB transceivers.

* **Digital Isolator (ADuM4160)**: Uses $iCoupler^\circledR$ planar magnetic transformers to isolate the USB data lines ($D+$ and $D-$).
* **Isolation Barrier Physical Design**: A continuous $6.0\text{ mm}$ physical creepage and clearance slot is routed through all PCB layers directly beneath the ADuM4160.
* **Plane Separation**: `GND_USB` (host reference) and `GND_INTERNAL` (joint motor/MCU reference) are physically separated. A single 1 nF / 2 kV 1812 ceramic capacitor spans the isolation barrier to provide a controlled high-frequency return path for common-mode RF noise without compromising DC galvanic isolation.

---

### 3.3 Sensor Alignment & Magnetic Field Parameters

The AS5600 operates by detecting the orientation of the horizontal magnetic field component ($B_\perp$) generated by a diametrically polarized cylindrical NdFeB magnet mounted to the motor rotor shaft.

* **Magnet Dimensions**: $\varnothing 6.0\text{ mm} \times 2.5\text{ mm}$ thickness, Grade N35SH (high-temperature tolerance up to $150^\circ\text{C}$).
* **Operating Airgap ($z$)**: $0.5\text{ mm} \le z \le 1.5\text{ mm}$ from the top surface of the SOIC-8 package.
* **Magnetic Field Target**: Between $45\text{ mT}$ and $90\text{ mT}$ at the sensing elements. The AS5600 automatic gain control (AGC) monitors field strength:
  * Register `AGC` (`0x1A`): Value between `0` and `255`. Nominal centered field yields `AGC` in the range $80 \le \text{AGC} \le 160$.
  * Register `STATUS` (`0x0B`):
    * Bit 5 (`MD`): Magnet detected.
    * Bit 4 (`ML`): Magnet too weak (airgap too large or magnet missing).
    * Bit 3 (`MH`): Magnet too strong (airgap too small).
* **Angular Resolution**:
  $$\text{LSB} = \frac{360^\circ}{2^{12}} = \frac{360^\circ}{4096} \approx 0.08789^\circ \approx 1.53398\text{ mrad}$$

---

### 3.4 Logic Level Shifting (TXS0108E)

The STM32F103C8T6 utilizes $3.3\text{V}$ CMOS levels. External motor driver optocouplers and Dynamixel TTL communication channels operate at $5.0\text{V}$. The TI TXS0108E provides bidirectional conversion:

* **Port A**: $V_{CCA} = 3.3\text{V}$ (MCU domain: `TIM2_CH1`, `PB1`, `PB2`, `USART1_TX`, `USART1_RX`).
* **Port B**: $V_{CCB} = 5.0\text{V}$ (Actuator domain: `STEP`, `DIR`, `ENABLE`, `DYNAMIXEL_DATA`).
* **Edge-Rate Accelerators**: Internal one-shot circuits detect rising edges and temporarily lower output impedance to charge capacitive loads (up to 70 pF) rapidly, sustaining data rates up to 20 Mbps on push-pull lines and up to 2 Mbps on open-drain lines.

---

## 4. Software/Hardware Interface

### 4.1 STM32F103 Pin Allocation & Peripheral Mapping

| MCU Pin    | Pin Name | Peripheral Function | Net Identifier | Connected Hardware / Function                      |
|:---------- |:-------- |:------------------- |:-------------- |:-------------------------------------------------- |
| **Pin 14** | `PA4`    | `GPIO_Output`       | `DYN_DIR_EN`   | TXS0108E Direction / Buffer Enable                 |
| **Pin 19** | `PB1`    | `GPIO_Output`       | `STEP_DIR`     | Stepper Direction control line                     |
| **Pin 20** | `PB2`    | `GPIO_Output`       | `STEP_EN`      | Stepper Output Stage Enable (Active-Low)           |
| **Pin 10** | `PA0`    | `TIM2_CH1`          | `STEP_PUL`     | Hardware Timer Step Pulse generation               |
| **Pin 30** | `PA9`    | `USART1_TX`         | `DYN_TX`       | Dynamixel Half-Duplex Transmit                     |
| **Pin 31** | `PA10`   | `USART1_RX`         | `DYN_RX`       | Dynamixel Half-Duplex Receive                      |
| **Pin 32** | `PA11`   | `USB_DM`            | `USB_DM`       | ADuM4160 Isolated USB Data Negative                |
| **Pin 33** | `PA12`   | `USB_DP`            | `USB_DP`       | ADuM4160 Isolated USB Data Positive                |
| **Pin 42** | `PB6`    | `I2C1_SCL`          | `AS5600_SCL`   | Magnetic Encoder Clock (400 kHz)                   |
| **Pin 43** | `PB7`    | `I2C1_SDA`          | `AS5600_SDA`   | Magnetic Encoder Data (400 kHz)                    |
| **Pin 2**  | `PC13`   | `GPIO_Output`       | `LED_STATUS`   | Active-Low Onboard Status/Debug LED                |
| **Pin 34** | `PA13`   | `SYS_JTMS-SWDIO`    | `SWDIO`        | Cortex SWD Data Debug Line                         |
| **Pin 37** | `PA14`   | `SYS_JTCK-SWCLK`    | `SWCLK`        | Cortex SWD Clock Debug Line                        |
| **Pin 44** | `BOOT0`  | `BOOT0`             | `BOOT0`        | Tied to GND via $10\text{ k}\Omega$ (Flash Boot)   |

---

### 4.2 Firmware Driver Implementation

#### AS5600 Register Polling & Health Check (HAL C++)

```cpp
#include "stm32f1xx_hal.h"

#define AS5600_I2C_ADDR         (0x36 << 1)
#define REG_STATUS              0x0B
#define REG_RAW_ANGLE_HIGH      0x0C
#define REG_RAW_ANGLE_LOW       0x0D

extern I2C_HandleTypeDef hi2c1;

typedef struct {
    uint16_t raw_angle;
    float radians;
    bool magnet_valid;
} JointEncoder_t;

bool AS5600_Update(JointEncoder_t *encoder) {
    uint8_t status = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, REG_STATUS, I2C_MEMADD_SIZE_8BIT, 
                         &status, 1, 10) != HAL_OK) {
        return false;
    }

    // Check MD (Magnet Detected) bit 5
    if (!(status & 0x20)) {
        encoder->magnet_valid = false;
        return false;
    }
    encoder->magnet_valid = true;

    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, REG_RAW_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, 
                         buf, 2, 10) == HAL_OK) {
        encoder->raw_angle = (((uint16_t)(buf[0] & 0x0F)) << 8) | buf[1];
        encoder->radians = ((float)encoder->raw_angle / 4096.0f) * 6.28318530718f;
        return true;
    }
    return false;
}
```

#### Hardware Timer Step-Frequency Generation

To generate step pulses without software interrupt jitter, `TIM2` is configured in PWM generation mode on Channel 1 (`PA0`):

```cpp
void Stepper_SetVelocity(uint32_t step_frequency_hz, uint8_t direction) {
    if (step_frequency_hz == 0) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        return;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // APB1 Timer Clock = 72 MHz
    // Set Prescaler to 71 -> 1 MHz counter clock (1 us tick)
    uint32_t timer_clock = 1000000;
    uint32_t period = timer_clock / step_frequency_hz;

    __HAL_TIM_SET_PRESCALER(&htim2, 71);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2); // 50% duty cycle

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
```

---

### 4.3 ROS 2 Control Pipeline & Communication Interface

The board presents itself to the NVIDIA Jetson AGX Orin as a High-Speed USB Virtual COM Port (`/dev/ttyACM*`). A custom `ros2_control` hardware interface (`arm_joint_hardware_interface`) coordinates state and command streaming at a deterministic 500 Hz update loop.

#### ROS 2 Interfaces & Topics

* **Trajectory Command Interface**:
  * Topic: `/arm/joint_<id>_controller/joint_trajectory`
  * Message: `trajectory_msgs/msg/JointTrajectory`
  * Action: `/arm/joint_trajectory_controller/follow_joint_trajectory`
  * Type: `control_msgs/action/FollowJointTrajectory`
* **Joint State Feedback Interface**:
  * Topic: `/joint_states`
  * Message: `sensor_msgs/msg/JointState`
  * Publishing Rate: 100 Hz aggregate (synced across all joints)
  * Transmitted fields: `name`, `position` (rad), `velocity` (rad/s), `effort` (Nm)
* **Diagnostic & Health Interface**:
  * Topic: `/arm/joint_<id>/diagnostics`
  * Message: `diagnostic_msgs/msg/DiagnosticArray`
  * Telemetry: AS5600 AGC level, packet checksum drop counters, thermal estimates, drive fault flags.

#### Packet Framing Protocol (USB CDC Serial)

Communications use binary packets framed by explicit header markers and CRC-16 validation:

```text
+--------+--------+---------+--------+------------------+---------+
| START1 | START2 | COMM_ID | LENGTH | PAYLOAD (N-byte) | CRC-16  |
|  0xAA  |  0x55  | (1 Byte)| (1 B)  |    (Variable)    | (2 Byte)|
+--------+--------+---------+--------+------------------+---------+
```

* Command Packet (`COMM_ID = 0x01`): Target Angle (float32, 4 bytes), Max Velocity (float32, 4 bytes), Torque Enable (uint8, 1 byte).
* Feedback Packet (`COMM_ID = 0x02`): Current Angle (float32, 4 bytes), Angular Velocity (float32, 4 bytes), AS5600 AGC (uint8, 1 byte), Fault Flags (uint8, 1 byte).

---

## 5. Future Work / Known Limitations

### 5.1 STM32F103 USB and CAN Peripheral Mutual Exclusion

On the STM32F103 architecture, the USB full-speed device peripheral and the CAN controller share a single dedicated 512-byte packet buffer SRAM. Furthermore, both peripherals map to the exact same interrupt vector line (`USB_LP_CAN1_RX0_IRQHandler`). 

Because of this architectural bottleneck, the board cannot run USB serial and CAN bus simultaneously on the current revision. While the MCP2551 CAN transceiver footprint was routed on the KiCad PCB (`ENCODER_USB.kicad_pcb`), it is currently depopulated in favor of USB CDC operation.

### 5.2 Transition to Multidrop CAN-FD Bus Topology

Running independent USB cables down an articulated 6-DOF arm creates a bulky cable bundle prone to torsional fatigue and connector disconnects. Future revisions (`ARM-PCB v2.0`) will implement the following improvements:

* **Controller Migration**: Transition from STM32F103 to STM32G431KB or STM32F405RG. These chips decouple the USB and CAN peripherals and integrate native CAN-FD (Flexible Data-Rate) controllers supporting payload bitrates up to 5 Mbps.
* **Daisy-Chained Topology**: Replace the star topology with a single 4-core hybrid umbilical:
  * `CAN_H` / `CAN_L` (twisted shielded pair, $120\;\Omega$ differential impedance)
  * `+24V_BUS` / `GND_PWR` ($18\;\text{AWG}$ high-flex silicone conductors)
* **Termination**: Implement software-switchable or jumper-selectable $120\;\Omega$ split-termination resistors at the terminal wrist node.

### 5.3 High Thermal Loss on Linear Regulators

The AMS1117-5.0 linear regulator dissipates approximately $0.70\text{ W}$ under typical quiescent load. In enclosed joint housings, this elevates local ambient temperature, causing thermal drift in the AS5600 Hall elements. Next-generation revisions will replace `U6` with a high-efficiency monolithic step-down buck converter (TI TPS54202 or LMR14006) switching at 400 kHz to reduce quiescent power loss by over 80%.

### 5.4 Encoder Resolution & High-Speed Noise Immunity

While the 12-bit AS5600 provides adequate resolution ($0.0879^\circ$) for primary joint link tracking, high-ratio gearboxes require higher precision before mechanical reduction. Future revisions will evaluate the **AMS AS5048A** (14-bit, $0.0219^\circ$ resolution) communicating over high-speed SPI with 16-bit CRC framing, significantly enhancing noise rejection against motor phase harmonics.
