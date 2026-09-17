# Power Distribution Board

## Overview & Objectives

The Power Distribution Board (PDB) serves as the central power routing module and low-level hardware controller for RoboManipal's RoboCup SML 2026 mobile base. The board interfaces a 24V nominal battery with high-current motion drives, the robotic manipulator, peripheral sensors, and the primary NVIDIA Jetson compute host.

```{figure} assets/amr_board_overview.png
:name: fig_amr_board_overview
:alt: Power Distribution PCB Top Layer Layout
:width: 80%

Central power distribution PCB layout illustrating domain separation and high-current trace zones.
```

### Core Engineering Objectives

* **Multi-Rail Power Regulation:** Regulate raw 24V battery input ($18.0\text{V} - 29.4\text{V}$ DC range) into filtered $12\text{V}$, $5\text{V}$, and $3.3\text{V}$ DC logic and sensor power buses.
* **Galvanic Domain Isolation:** Prevent motor switching noise, back-EMF spikes, and ground bounce from corrupting sensor signals and microcontrollers through dual electrical domains.
* **Real-Time Motion & Telemetry Polling:** Offload time-critical quadrature encoder processing and IMU sampling to dedicated microcontrollers, freeing host compute cycles.
* **Actuation Power Gating:** Maintain active host compute state estimation and network links while providing dedicated mechanical emergency stop switches for motion actuators.

---

## Technical Specifications & Bill of Materials (BOM)

### System Specifications

| Parameter | Value / Specification | Engineering Constraint |
| --- | --- | --- |
| **Input Bus Voltage** | $24.0\text{V}$ Nominal ($18.0\text{V} - 29.4\text{V}$ operational) | 7S Li-ion / 8S LiFePO4 battery pack |
| **Peak System Current** | $45\text{A}$ Total ($30\text{A}$ Base Motors + $15\text{A}$ Manipulator) | Continuous copper thermal limit |
| **Base Drive Supply Rail** | $24.0\text{V} \pm 0.5\text{V} @ 30\text{A}$ max | Fused via Base Kill switch |
| **Manipulator Supply Rail** | $24.0\text{V} \pm 0.5\text{V} @ 15\text{A}$ max | Fused via Arm Kill switch |
| **Logic Buck Converter Output** | $5.0\text{V} @ 5\text{A}$ ($25\text{W}$), $12.0\text{V} @ 3\text{A}$ ($36\text{W}$) | Down-stepped from raw 24V bus |
| **Isolated Logic Converter** | Mornsun B2405S-2WR3 ($24\text{V}$ to $5\text{V}$, $2\text{W}$, $3\text{kVDC}$ Isolation) | Powers Domain A control side |
| **Primary Controllers** | 2x STM32F411CEU6 ($100\text{ MHz}$ ARM Cortex-M4, $512\text{KB}$ Flash) | UFQFPN-48 package |
| **Host Interface** | USB 2.0 Full-Speed ($12\text{ Mbps}$) Virtual COM (CDC) | Native STM32-1 USB transceiver |
| **Inter-Domain Link** | 3x 6N137 High-Speed Optocouplers ($10\text{ MBd}$) | 2x UART + 1x Hardware Sync Clock |

### Bill of Materials (BOM)

| Designator | Component / Part Number | Manufacturer | Package / Case | Key Specs & Purpose |
| --- | --- | --- | --- | --- |
| **MCU1, MCU2** | STM32F411CEU6 | STMicroelectronics | UFQFPN-48 ($7\times7\text{ mm}$) | Microcontroller ($100\text{ MHz}$, $125\text{ DMIPS}$) |
| **U1** | Mornsun B2405S-2WR3 | Mornsun | SIP-4 ($11.6\times6.0\text{ mm}$) | $24\text{V}$-to-$5\text{V}$ $2\text{W}$ $3\text{kVDC}$ Isolated DC-DC |
| **U2, U3** | LM2596S-ADJ | Texas Instruments | TO-263-5 (D2PAK) | Step-Down Voltage Regulator ($3\text{A}$ output) |
| **U4, U5, U6** | 6N137 | Vishay / ON Semi | SOIC-8 | Single-Channel $10\text{ MBd}$ Optocoupler |
| **F1, F2** | MF-MSMF050-2 | Bourns | SMD 1812 | $500\text{mA}$ Hold / $1.0\text{A}$ Trip PTC Polyfuse |
| **F3** | MF-MSMF020-2 | Bourns | SMD 1812 | $200\text{mA}$ Hold / $400\text{mA}$ Trip PTC Polyfuse |
| **D1, D2, D3** | SMBJ24A | Littelfuse | DO-214AA (SMB) | $600\text{W}$ $24\text{V}$ Unidirectional TVS Diode |
| **SW1** | Panel E-Stop ($250\text{V}/10\text{A}$) | Indus | $22\text{ mm}$ Cutout | Main System Power Disconnect Switch |
| **SW2, SW3** | Heavy-Duty Toggle ($30\text{A}$) | KCD4 | Panel Mount | Base Drive / Arm Power Gating Switches |

---

## Design & Architecture

```{figure} assets/amr_electrical_architecture.png
:name: fig_electrical_arch
:alt: Complete Electrical Architecture Diagram
:width: 90%

Block diagram detailing power distribution, dual MCU domains, and isolation boundaries.
```

### Dual-STM Isolation Scheme

To eliminate motor PWM switching noise ($20\text{ kHz}$) and ground loops from corrupting low-amplitude sensor readings, the board splits control logic into two isolated domains:

```{figure} assets/dual_stm_topology.png
:name: fig_dual_stm_topology
:alt: Optocoupler Inter-Domain Communication Schematic
:width: 85%

Schematic layout of cross-domain UART and hardware clock synchronization across the optical barrier.
```

#### Domain A (Isolated Compute & Actuation Bridge)

* **Power Source:** Mornsun B2405S-2WR3 isolated DC-DC converter output ($5\text{V}_{\text{ISO}}$ stepped to $3.3\text{V}_{\text{ISO}}$ via LDO).
* **Controller:** **STM32-1**.
* **Peripherals:** Base motor quadrature encoders (Channels A/B), USB CDC link to NVIDIA Jetson host, status indicators.
* **Ground Reference:** Isolated Signal Ground ($\text{GND}_{\text{ISO}}$).

#### Domain B (Raw Power & Sensor Acquisition)

* **Power Source:** Non-isolated $5\text{V}/3.3\text{V}$ buck regulator supplied from raw 24V bus.
* **Controller:** **STM32-2**.
* **Peripherals:** MPU6050 / ICM-20948 9-DOF IMU module over SPI1 interface, auxiliary thermal sensors.
* **Ground Reference:** Raw Power Ground ($\text{GND}_{\text{PWR}}$).

#### Cross-Domain Communication Interface

The two domains exchange structured binary packets over an optically isolated interface consisting of three 6N137 optocouplers:

* **Tx/Rx Data Channels:** 2x UART lines operating at $115200\text{ baud}$ ($8\text{N}1$ configuration).
* **Hardware Sync Line:** 1x clock line pulsed by STM32-1 at $100\text{ Hz}$ to trigger simultaneous IMU sensor sampling on STM32-2 and encoder hardware counter latching on STM32-1.

```
+-------------------------------------------------------------------------+
|                              DOMAIN A (ISO)                             |
|  +--------------+   USB CDC   +---------+   TIM2/TIM3   +------------+  |
|  | Jetson Host  |<===========>| STM32-1 |<------------->| Encoders   |  |
|  +--------------+             +---------+               +------------+  |
+------------------------------------|------------------------------------+
                                     | 6N137 Optocouplers
                                     | (1x CLK Sync, 2x UART)
+------------------------------------|------------------------------------+
|                              DOMAIN B (RAW)                             |
|                               +---------+     SPI1      +------------+  |
|                               | STM32-2 |<------------->| 9-DOF IMU  |  |
|                               +---------+               +------------+  |
+-------------------------------------------------------------------------+
```

### Power Rail & Switch Topology

```{figure} assets/switch_schematic.png
:name: fig_switch_schematic
:alt: Power Switch and Fuse Distribution Topology
:width: 85%

Power distribution schematic showing kill switch placement and downstream protection fuses.
```

```{figure} assets/polyfuse_schematic.png
:name: fig_polyfuse_schematic
:alt: Resettable PTC Polyfuse Protection Topology
:width: 80%

Schematic layout of resettable PTC polyfuses protecting logic voltage rails and IMU sensor power lines.
```

The 24V bus routes through a triple-switch topology designed for field safety during competition operations:

1. **Main Kill Switch (`SW1`):** In-line with the primary positive battery wire ($10\text{ AWG}$). Disconnects the battery from all onboard systems.
2. **Base Kill Switch (`SW2`):** Gates the $24\text{V}$ power line to the motor drive H-bridges ($30\text{A}$ rating).
3. **Arm Kill Switch (`SW3`):** Gates the $24\text{V}$ power line to the robotic manipulator controller ($15\text{A}$ rating).

```{important}
**NVIDIA Jetson Power Retention:** The NVIDIA Jetson host power supply is tapped directly downstream of the **Main Kill Switch (`SW1`)**, but upstream of **`SW2`** and **`SW3`**. Disabling base motion or arm movement via `SW2`/`SW3` keeps the Jetson alive, maintaining active state estimation, SLAM processing, and network diagnostic telemetry over Wi-Fi/Ethernet.
```

### High-Current PCB Routing & Thermal Design

```{figure} assets/pcb_unmasked_traces.png
:name: fig_pcb_unmasked
:alt: High Current Unmasked Copper Traces Layout
:width: 75%

PCB layout showing unmasked copper trace regions prepared for post-assembly solder deposition.
```

To carry $30\text{A}$ continuous current without exceeding the $30^\circ\text{C}$ PCB trace temperature rise limit on standard $1\text{ oz/ft}^2$ ($35\,\mu\text{m}$) copper thickness, high-current paths are designed using intentionally unmasked copper rails.

#### IPC-2221 Trace Width Calculation

Using the IPC-2221 external layer current capacity formula:

$$I = k \cdot \Delta T^{0.44} \cdot A^{0.725}$$

Where:

* $I = 30\text{ A}$ (Maximum continuous current)
* $k = 0.048$ (External layer constant)
* $\Delta T = 30^\circ\text{C}$ (Allowable temperature rise)
* $A$ = Cross-sectional area in $\text{mil}^2$

Solving for trace area $A$:

$$A = \left( \frac{30}{0.048 \cdot (30)^{0.44}} \right)^{\frac{1}{0.725}} \approx 523.6\text{ mil}^2$$

For a standard $1\text{ oz/ft}^2$ ($1.37\text{ mil}$ / $35\,\mu\text{m}$) copper foil, the required trace width $W$ would be:

$$W = \frac{A}{\text{Thickness}} = \frac{523.6\text{ mil}^2}{1.37\text{ mil}} \approx 382\text{ mil} \ (9.7\text{ mm})$$

#### Unmasked Copper Reinforcement

To conserve PCB surface area, high-power traces are routed at a width of $200\text{ mil}$ ($5.08\text{ mm}$) with the top solder mask layer opened over the trace. Post-reflow manual solder filling adds a $\sim 150\,\mu\text{m}$ layer of $63/37 \text{ Sn/Pb}$ solder ($R_{\text{solder}} = 14.5\times10^{-8}\,\Omega\cdot\text{m}$), reducing equivalent trace resistance:

$$R_{\text{total}} = \left( \frac{1}{R_{\text{Cu}}} + \frac{1}{R_{\text{Solder}}} \right)^{-1}$$

This modification increases overall current capacity to $>40\text{A}$ while holding trace temperature rise under $18^\circ\text{C}$ at $30\text{A}$ continuous load.

---

## Software/Hardware Interface

### Pinout & Peripheral Mapping

#### STM32-1 (Domain A - Isolated Compute Bridge & Encoders)

| Pin | Peripheral / Signal | Connected Target | Protocol / Mode |
| --- | --- | --- | --- |
| **PA11** | USB_DM | NVIDIA Jetson AGX Orin | USB 2.0 Full-Speed Data Direct |
| **PA12** | USB_DP | NVIDIA Jetson AGX Orin | USB 2.0 Full-Speed Data Direct |
| **PA2** | USART2_TX | Optocoupler U4 (Tx to STM32-2) | $115200\text{ baud}$, $8\text{N}1$ |
| **PA3** | USART2_RX | Optocoupler U5 (Rx from STM32-2) | $115200\text{ baud}$, $8\text{N}1$ |
| **PB0** | GPIO_Output | Optocoupler U6 (Sync Out) | Push-Pull, $100\text{ Hz}$ Pulse Output |
| **PA0** | TIM2_CH1 | Left Base Encoder (Channel A) | Quadrature Encoder Mode |
| **PA1** | TIM2_CH2 | Left Base Encoder (Channel B) | Quadrature Encoder Mode |
| **PB4** | TIM3_CH1 | Right Base Encoder (Channel A) | Quadrature Encoder Mode |
| **PB5** | TIM3_CH2 | Right Base Encoder (Channel B) | Quadrature Encoder Mode |

#### STM32-2 (Domain B - Raw Power Domain & IMU Interface)

| Pin | Peripheral / Signal | Connected Target | Protocol / Mode |
| --- | --- | --- | --- |
| **PA9** | USART1_TX | Optocoupler U5 (Tx to STM32-1) | $115200\text{ baud}$, $8\text{N}1$ |
| **PA10** | USART1_RX | Optocoupler U4 (Rx from STM32-1) | $115200\text{ baud}$, $8\text{N}1$ |
| **PB1** | GPIO_EXTI1 | Optocoupler U6 (Sync In) | Falling Edge Interrupt Input |
| **PA5** | SPI1_SCK | ICM-20948 IMU | $8\text{ MHz}$ SPI Master Clock |
| **PA6** | SPI1_MISO | ICM-20948 IMU | SPI Data Input |
| **PA7** | SPI1_MOSI | ICM-20948 IMU | SPI Data Output |
| **PB2** | GPIO_Output | ICM-20948 CS | Chip Select (Active Low) |

### Signal Protection & ESD Suppression

```{figure} assets/tvs_diode_schematic.png
:name: fig_tvs_diode_schematic
:alt: TVS Diode Array ESD Protection Schematic
:width: 85%

USBLC6-4SC6 low-capacitance TVS diode arrays protecting encoder lines, motor control signals, and I2C lines against ESD and inductive transients.
```

### Inter-MCU Protocol & Telemetry Data Flow

```
+------------------+     USB CDC (Virtual COM Port)     +-------------------+
|  NVIDIA Jetson   |<---------------------------------->| /dev/ttyACM0      |
|  AGX Orin Host   |                                    | (STM32-1 Domain A)|
+------------------+                                    +-------------------+
        ^                                                         ^
        | Transmit: Velocity Commands                             | Receive: Enc + IMU
        v                                                         v
+------------------+       Optocouplers (Isolated)       +-------------------+
|  Base Motor PWM  |<-----------------------------------| STM32-2 Domain B  |
|  Drive Signal    |      2x UART @ 115200 + 1x CLK     | (IMU Polling)     |
+------------------+                                     +-------------------+
```

---

## Future Work / Known Limitations

### Identified Hardware Limitations

* **Optocoupler Throughput Bottleneck:** The 6N137 optocoupler propagation delay caps the inter-MCU UART baud rate to $115200\text{ baud}$, restricting raw IMU streaming rate to $\le 200\text{ Hz}$.
* **Manual Solder Process Sensitivity:** Manual solder-bridging over unmasked trace runs causes unit-to-unit trace resistance variation ($\pm 12\%$), requiring individual board thermal verification.
* **Passive Overcurrent Detection:** PTC polyfuses provide hardware trip protection but do not provide analog current telemetry back to the Jetson host prior to tripping.

### Next-Iteration Upgrades

* **Digital Capacitive Isolators:** Replace 6N137 optocouplers with Texas Instruments ISO7741 digital isolators ($100\text{ Mbps}$ throughput, $5\text{ kV}_{\text{RMS}}$ isolation) to allow $2\text{ MBaud}$ SPI bridging across domains.
* **Active Power Monitoring:** Integrate INA226 $16\text{-bit}$ $I^2C$ current/voltage/power monitors upstream of `SW2` and `SW3` to publish live telemetry to the main diagnostic bus.
* **Single-STM32 Topology with I2C Isolators:** Consolidate control logic to a single STM32F411 residing on Domain A (Isolated). Bridge Domain B sensor buses (IMU, current monitors) across the galvanic barrier using dedicated bidirectional I2C digital isolators (e.g., Texas Instruments ISO1540 / Analog Devices ADuM1250). This eliminates inter-MCU UART synchronization protocols, cuts MCU BOM costs, and unifies firmware execution into a single context while supporting up to $1\text{ MHz}$ I2C clock speeds.
