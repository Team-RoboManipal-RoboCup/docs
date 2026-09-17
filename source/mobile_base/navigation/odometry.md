# Dead-Wheel Odometry & Serial Bridge

This module documents the real-time planar pose estimation and serial bridge architecture for the RoboCup Smart Manufacturing League (SML) 2026 mobile base. The system streams raw quadrature encoder ticks from an STM32 microcontroller over a high-speed USB CDC serial link into ROS 2, computes dead-reckoning kinematics, and broadcasts continuous coordinate transforms for downstream SLAM and Nav2 navigation.

---

## 1. Overview & Objectives

The tracking assembly isolates odometry from drivetrain wheel slip using a **three-wheel dead-wheel odometry configuration**. Spring-loaded omnidirectional tracking wheels follow floor contours independently of the driven mecanum wheels.

### Primary Objectives
* **High-Rate Tick Streaming:** Ingest raw encoder tick counts from the STM32 microcontroller to ROS 2 at 100 Hz over a robust USB serial link.
* **Kinematic Integration:** Invert three-wheel dead-wheel encoder increments into continuous body-frame displacements ($\Delta x_{\text{body}}, \Delta y_{\text{body}}, \Delta \theta$) and integrate them into world-frame pose $(x, y, \theta)$.
* **Coordinate Frame Broadcasting:** Publish the `odom → base_link` TF transform to provide a continuous reference for the EKF, SLAM Toolbox, and Nav2 controllers.

---

## 2. Technical Specifications

### Kinematic & Geometric Parameters

```{image} ../assets/DeadWheel.png
:alt: Three-Wheel Dead-Wheel Odometry Geometry
:width: 480px
:align: center
```
*Figure 1: Geometric layout of the three-wheel dead-wheel tracking system.*

| Parameter | Symbol / Variable | Value | Description |
| :--- | :--- | :--- | :--- |
| **Wheelbase Length Constant** | $l$ | $0.255\text{ m}$ | Lateral separation between parallel tracking wheels |
| **Lateral Offset Constant** | $b$ | $0.186\text{ m}$ | Longitudinal distance from tracking center to lateral wheel |
| **Tracking Wheel Radius** | $r$ | $0.0295\text{ m}$ | 59 mm nominal omni wheel diameter |
| **Encoder Resolution** | $N$ | $2400\text{ ticks/rev}$ | Optical quadrature encoder line count |
| **Linear Distance per Tick** | $c = \frac{2\pi r}{N}$ | $\approx 7.72 \times 10^{-5}\text{ m/tick}$ | Conversion constant from raw ticks to linear displacement |
| **Serial Baud Rate** | — | $115200\text{ bps}$ | USB CDC Virtual COM Port baud rate |
| **Serial Device Path** | — | `/dev/ttyACM0` | Linux serial tty device assigned to STM32 |
| **Telemetry Poll Rate** | — | $100\text{ Hz}$ ($0.01\text{ s}$) | Asynchronous timer loop rate for serial reads |

### Embedded & Host Compute Allocation

| Component | Hardware Platform | Functional Role |
| :--- | :--- | :--- |
| **Tracking MCU** | STM32F411CEU6 (ARM Cortex-M4) | Acquires hardware timer encoder pulses; formats JSON-framed serial packets; receives outbound commands. |
| **Onboard ROS 2 Host** | NVIDIA Jetson AGX Orin | Executes `serial_bridge` and `smart_brain` ROS 2 nodes in Python (`rclpy`). |

---

## 3. Design & Architecture

### 3.1 Data Flow Pipeline

The end-to-end odometry pipeline connects embedded timer peripherals with the ROS 2 transform buffer:

```{mermaid}
flowchart LR
    STM32["STM32 MCU<br/>Encoder Ticks, JSON Framing"] -->|USB Serial 115200| SB["serial_bridge Node"]
    SB -->|/wheel_tick| BR["smart_brain Node"]
    BR -->|TF: odom to base_link| TF["tf2 Buffer"]
    SB -->|Raw Command String| STM32
```

### 3.2 Encoder-to-Wheel Mapping

The STM32 encodes tick counters into a JSON dictionary prefixed with `E`. The keys `{a, b, c}` are ingested by `serial_bridge` and mapped into a standard `[left, lateral, right]` triple published on `/wheel_tick`:

```python
# serial_bridge node: unpacking serial JSON
n1, n2, n3 = float(data['a']), float(data['b']), float(data['c'])
msg.data = [n1, n2, n3]  # Published as [left, lateral, right]
```

Within `smart_brain`, delta ticks are calculated across consecutive sampling intervals with coordinate inversion applied to the lateral wheel:

```python
# smart_brain node: delta computation
dn1 = msg.data[2] - self.n1       # Right tracking wheel
dn2 = msg.data[0] - self.n2       # Left tracking wheel
dn3 = (-msg.data[1]) - self.n3    # Lateral tracking wheel (sign-inverted)
```

### 3.3 Kinematics Formulation

The three-wheel configuration derives body-frame translation and rotation from delta tick counts per cycle:

#### Body-Frame Displacements

$$
\Delta x_{\text{body}} = -c \cdot \frac{\Delta n_1 + \Delta n_2}{2}
$$

$$
\Delta y_{\text{body}} = c \left( \Delta n_3 - (\Delta n_1 - \Delta n_2)\frac{b}{l} \right)
$$

$$
\Delta \theta = \frac{c}{l}(\Delta n_2 - \Delta n_1)
$$

#### World-Frame Integration

Body-frame displacements are rotated into the global fixed odometry reference frame via standard 2D planar rotation:

$$
x_{k+1} = x_k + \Delta x_{\text{body}}\cos\theta_k - \Delta y_{\text{body}}\sin\theta_k
$$

$$
y_{k+1} = y_k + \Delta x_{\text{body}}\sin\theta_k + \Delta y_{\text{body}}\cos\theta_k
$$

$$
\theta_{k+1} = \theta_k + \Delta \theta
$$

Heading $\theta$ is converted into an orientation quaternion using planar ZYX Euler angles ($\text{roll} = \text{pitch} = 0$):

$$
q_z = \sin\left(\frac{\theta}{2}\right), \qquad q_w = \cos\left(\frac{\theta}{2}\right), \qquad q_x = q_y = 0
$$

### 3.4 Node Responsibilities

| Node | File / Implementation | Responsibilities |
| :--- | :--- | :--- |
| **`serial_bridge`** | `serial_bridge.py` | Owns `/dev/ttyACM0`; parses inbound `E{...}` strings into `/wheel_tick` and `/imu/data_raw`; forwards outbound strings from `/mcu/out` to serial. |
| **`smart_brain`** | `smart_brain.py` | Consumes `/wheel_tick`; evaluates forward kinematics; broadcasts continuous `odom → base_link` TF transform. |

---

## 4. Software & Hardware Interface

### ROS 2 Topics

| Topic Name | Message Type | Direction | Node | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/wheel_tick` | `std_msgs/Float64MultiArray` | Publish | `serial_bridge` | Array containing `[left, lateral, right]` accumulated encoder ticks. |
| `/mcu/out` | `std_msgs/String` | Subscribe | `serial_bridge` | Raw command strings forwarded directly to the STM32 as `<data>\r\n`. |
| `/odom` | `nav_msgs/Odometry` | Publish | `smart_brain` | Optional odometry message topic (instantiated for compatibility). |
| `odom → base_link` | `tf2_msgs/TFMessage` | Broadcast | `smart_brain` | Rigid coordinate transform updated on every `/wheel_tick` callback. |

### Serial Protocol Specification

| Field | Configuration | Notes |
| :--- | :--- | :--- |
| **Physical Port** | `/dev/ttyACM0` | Linux USB CDC ACM serial link |
| **Baud Rate & Settings** | 115200 baud, 8-N-1 | 8 data bits, no parity, 1 stop bit, 0.1 s read timeout |
| **Inbound Frame Header** | `E{` | Lines starting with `E{` contain JSON payload: `{"a":..., "b":..., "c":...}` |
| **Outbound Frame Format** | `<command>\r\n` | Appends carriage return and newline delimiters |
| **Exception Handling** | Silent Drop | Malformed frames or checksum anomalies are safely caught and dropped without crashing the serial thread |
