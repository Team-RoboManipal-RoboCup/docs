# Sensor Fusion (Dead-Wheels & IMU)

This module details the planar odometry state estimation system for the RoboManipal RoboCup SML 2026 mobile base. The system fuses a 3-deadwheel spring-loaded tracking assembly with a CEVA/Hillcrest Labs BNO085 IMU (Game Rotation Vector and 3-axis linear acceleration) to estimate robot planar pose $\mathbf{x} = [x, y, \theta]^T$ and velocities $\mathbf{v} = [v_x, v_y, \omega]^T$ at 50 Hz.

---

## 1. Overview & Objectives

The primary objective of the odometry pipeline is providing drift-minimised, slip-immune planar state estimation during high-acceleration maneuvers across warehouse flooring. The robot base is driven by 4 independent motors governed by Cytron MD30C drivers, introducing substantial mechanical wheel slip during rapid trajectory tracking.

To decouple state estimation from drive wheel slip and gear backlash, tracking is performed via three unpowered omni-directional deadwheels held under constant spring preload against the driving surface. Orientation and dynamic translation are reinforced by fusing a CEVA/Hillcrest BNO085 IMU running in 6-DOF **Game Rotation Vector** mode, completely avoiding the catastrophic electromagnetic disturbances induced by motor drive currents on magnetometers.

```{mermaid}
flowchart LR
    subgraph Base_Hardware ["Base Hardware"]
        DW["3x Spring-Loaded Deadwheels"] -->|Quadrature Pulses| STM_TIM["STM32F411 Timers: TIM4, TIM3, TIM2"]
        BNO["BNO085 IMU: SH-2"] -->|I2C3 at 100 kHz| STM_I2C["STM32 I2C3 SHTP Driver"]
    end

    subgraph STM32_Firmware ["STM32F411 Real-Time Layer"]
        STM_TIM --> ENC_MOD["encoder_update Counts: e1, e2, e3"]
        STM_I2C --> IMU_MOD["bno085_driver Game Yaw and Accel"]
        ENC_MOD --> CDC_TX["usbd_cdc_if 50 Hz JSON Serial"]
        IMU_MOD --> CDC_TX
    end

    subgraph Jetson_Compute ["Nvidia Jetson State Estimation Layer"]
        CDC_TX -->|USB CDC Port| SER_RX["Serial Receiver Node"]
        SER_RX --> KIN["Deadwheel Forward Kinematics"]
        KIN --> KF["Kalman Filter: EKF State Fusion"]
        SER_RX --> KF
        KF --> ODOM["/odom Odometry Msg"]
        KF --> TF["TF: odom to base_footprint"]
    end
```

---

## 2. Technical Specifications & Bill of Materials (BOM)

### 2.1 Hardware Bill of Materials

| Component | Part Number / Model | Quantity | Operating Specs | Interface / Bus |
| :--- | :--- | :--- | :--- | :--- |
| **Microcontroller Unit** | STM32F411CEUx | 1 | ARM Cortex-M4 @ 96 MHz, 512 KB Flash, 128 KB SRAM | USB OTG FS, Hardware Timers |
| **Inertial Measurement Unit** | CEVA / Hillcrest BNO085 | 1 | 3-axis Accel ($\pm 16\text{g}$), 3-axis Gyro ($\pm 2000^\circ/\text{s}$), Cortex-M0+ SH-2 core | I2C3 (100 kHz), Address `0x4A` |
| **Deadwheel Tracking Pods** | 58 mm Double Omni-Wheel Pod | 3 | High-traction rubber rollers, spring suspension | Mechanical ground contact |
| **Deadwheel Optical Encoders**| US Digital / Broadcom Quadrature | 3 | 2048 CPR (8192 counts/rev in $4\times$ decoding) | Differential RS422 / TTL to STM32 |
| **Main Compute Engine** | Nvidia Jetson Orin Nano / Xavier | 1 | 6-core ARM CPU, Ubuntu 22.04 LTS, ROS 2 Humble | USB 2.0 / 3.0 Host to STM32 CDC |

### 2.2 Pinout & Hardware Allocation

| Signal Name | MCU Pin | Peripheral Function | Configuration Mode | Connected Device |
| :--- | :--- | :--- | :--- | :--- |
| `ENC1_A` | `PB6` | `TIM4_CH1` | Encoder Mode (`TIM_ENCODERMODE_TI12`), Filter = 5 | Deadwheel 1 Channel A |
| `ENC1_B` | `PB7` | `TIM4_CH2` | Encoder Mode (`TIM_ENCODERMODE_TI12`), Filter = 5 | Deadwheel 1 Channel B |
| `ENC2_A` | `PB4` | `TIM3_CH1` | Encoder Mode (`TIM_ENCODERMODE_TI12`), Filter = 5 | Deadwheel 2 Channel A |
| `ENC2_B` | `PB5` | `TIM3_CH2` | Encoder Mode (`TIM_ENCODERMODE_TI12`), Filter = 5 | Deadwheel 2 Channel B |
| `ENC3_B` | `PA15`| `TIM2_CH1` | Encoder Mode (`TIM_ENCODERMODE_TI12`), Filter = 5 | Deadwheel 3 Channel B |
| `ENC3_A` | `PB3` | `TIM2_CH2` | Encoder Mode (`TIM_ENCODERMODE_TI12`), Filter = 5 | Deadwheel 3 Channel A |
| `I2C3_SCL` | `PA8` | `I2C3_SCL` | Alternate Function 4 (`GPIO_AF4_I2C3`), Open-Drain | BNO085 SCL (Clock) |
| `I2C3_SDA` | `PB8` | `I2C3_SDA` | Alternate Function 9 (`GPIO_AF9_I2C3`), Open-Drain | BNO085 SDA (Data) |
| `USB_DM` | `PA11`| `OTG_FS_DM` | Alternate Function 10 (`GPIO_AF10_OTG_FS`) | USB CDC D- |
| `USB_DP` | `PA12`| `OTG_FS_DP` | Alternate Function 10 (`GPIO_AF10_OTG_FS`) | USB CDC D+ |

---

## 3. Design & Architecture

### 3.1 BNO085 Game Rotation Vector vs. Magnetometer Vulnerability

The mobile base uses 4 MD30C motor drivers switching motor bus voltages up to 24V under high PWM duty cycles. When the drive motors draw dynamic currents ranging from $2\text{ A}$ to $30\text{ A}$, the resulting electromagnetic interference (EMI) produces transient magnetic flux densities of hundreds of microteslas ($\mu\text{T}$) at the chassis level. 

Furthermore, structural steel screws, steel bearings, and battery pack casings generate severe **hard-iron** (permanent bias offset) and **soft-iron** (flux skewing) distortions.

:::{warning}
Attempting to run the standard BNO085 9-DOF Rotation Vector (`BNO_REPORT_ROTATION`, `0x05`) subjects the internal extended Kalman filter of the SH-2 sensor hub to violent magnetometer measurement jumps whenever motors start, accelerate, or reverse. This causes false yaw rotations up to $45^\circ$ within fractions of a second, causing Kalman filter divergence.
:::

To solve this, the firmware exclusively activates the **Game Rotation Vector** (`BNO_REPORT_GAME_ROTATION`, `0x08`) at 50 Hz ($20000\,\mu\text{s}$ report interval) via `BNO085_EnableGameRotation()`:

1. **Pure 6-DOF Fusion**: Fuses exclusively the 3-axis MEMS accelerometer and 3-axis MEMS gyroscope.
2. **Tilt Compensation**: The gravity vector measured by the accelerometer continuously references pitch and roll, preventing vertical attitude drift.
3. **Drift-Isolated Heading**: The yaw axis relies strictly on numerical integration of the calibrated Z-gyroscope rate. The magnetometer input is disconnected inside the SH-2 fusion engine.
4. **Zero Heading Reference**: The yaw angle begins relative to the heading of the robot at boot ($0^\circ$). Because planar dead reckoning requires relative differential orientation rather than true geographic North, the Game Rotation Vector delivers an ultra-smooth, spike-free yaw angle $\theta_{\text{imu}}$.

### 3.2 3-Deadwheel Kinematics

The tracking assembly consists of three omni-directional deadwheels positioned around the geometric center of the robot. Let each deadwheel $i \in \{1, 2, 3\}$ have radius $r$, encoder resolution $N_{\text{cpr}}$ ticks per revolution, located at radial distance $L_i$ from the robot center of rotation with wheel orientation angle $\alpha_i$ relative to the robot's local X-axis (forward).

```{mermaid}
flowchart TD
    subgraph Kinematic_Mapping ["Deadwheel Kinematic Mapping"]
        RAW["Raw Timer Counters e1, e2, e3"] --> DELTA["Delta Counts: delta_e"]
        DELTA --> DISP["Wheel Displacements: delta_s"]
        DISP --> MAT["Kinematic Inversion: Jacobian Inversion"]
        MAT --> VEL["Deadwheel Planar Velocities: vx, vy, omega"]
    end
```

The linear travel $\Delta s_i$ of wheel $i$ during timestep $\Delta t$ is computed from the encoder delta $\Delta e_i$ as:

$$
\Delta s_i = \frac{2 \pi r}{N_{\text{cpr}}} \Delta e_i
$$

For a robot planar velocity vector in the robot frame $\mathbf{v}_r = \begin{bmatrix} v_{xr} & v_{yr} & \omega_r \end{bmatrix}^T$, the tangential velocity at each deadwheel $i$ is given by:

$$
v_i = -\sin(\alpha_i) v_{xr} + \cos(\alpha_i) v_{yr} + L_i \omega_r
$$

Assembling the geometric Jacobian $\mathbf{J}_{\text{dw}} \in \mathbb{R}^{3 \times 3}$:

$$
\begin{bmatrix} v_1 \\ v_2 \\ v_3 \end{bmatrix} = \mathbf{J}_{\text{dw}} \begin{bmatrix} v_{xr} \\ v_{yr} \\ \omega_r \end{bmatrix} = \begin{bmatrix} -\sin(\alpha_1) & \cos(\alpha_1) & L_1 \\ -\sin(\alpha_2) & \cos(\alpha_2) & L_2 \\ -\sin(\alpha_3) & \cos(\alpha_3) & L_3 \end{bmatrix} \begin{bmatrix} v_{xr} \\ v_{yr} \\ \omega_r \end{bmatrix}
$$

Inverting the non-singular Jacobian matrix yields the deadwheel-derived robot planar velocities:

$$
\mathbf{v}_r = \mathbf{J}_{\text{dw}}^{-1} \begin{bmatrix} \frac{\Delta s_1}{\Delta t} \\ \frac{\Delta s_2}{\Delta t} \\ \frac{\Delta s_3}{\Delta t} \end{bmatrix}
$$

### 3.3 Kalman Filter State-Space Formulation

The sensor fusion algorithm runs on the Nvidia Jetson at $\Delta t = 0.02\text{ s}$ ($50\text{ Hz}$). The state vector $\mathbf{x}_k \in \mathbb{R}^6$ represents 2D pose and velocities in the fixed odometry frame (`odom`):

$$
\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \\ \dot{x}_k \\ \dot{y}_k \\ \dot{\theta}_k \end{bmatrix}
$$

#### 3.3.1 Process Model (Prediction)

Assuming a constant-velocity kinematic motion model driven by continuous acceleration white noise $\mathbf{w}(t) = [w_{\ddot{x}}, w_{\ddot{y}}, w_{\ddot{\theta}}]^T$:

$$
\mathbf{x}_k = \mathbf{F}_k \mathbf{x}_{k-1} + \mathbf{w}_k
$$

The state transition matrix $\mathbf{F}_k \in \mathbb{R}^{6 \times 6}$ is formulated as:

$$
\mathbf{F}_k = \begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

The state prediction and covariance propagation steps are:

$$
\hat{\mathbf{x}}_k^- = \mathbf{F}_k \hat{\mathbf{x}}_{k-1}
$$

$$
\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

#### 3.3.2 Measurement Model (Correction)

The measurement vector $\mathbf{z}_k \in \mathbb{R}^6$ consists of the deadwheel-calculated velocities mapped to the global frame, the BNO085 Game Rotation Vector yaw, and the 2D linear accelerations rotated into the global frame:

$$
\mathbf{z}_k = \begin{bmatrix} 
\dot{x}_{\text{enc}, k} \\ 
\dot{y}_{\text{enc}, k} \\ 
\dot{\theta}_{\text{enc}, k} \\ 
\theta_{\text{imu}, k} \\ 
\ddot{x}_{\text{imu}, k} \\ 
\ddot{y}_{\text{imu}, k} 
\end{bmatrix}
$$

The deadwheel local velocities $[v_{xr}, v_{yr}]^T$ are transformed into the world frame via the predicted yaw $\hat{\theta}_k^-$:

$$
\begin{bmatrix} \dot{x}_{\text{enc}} \\ \dot{y}_{\text{enc}} \end{bmatrix} = \begin{bmatrix} \cos(\hat{\theta}_k^-) & -\sin(\hat{\theta}_k^-) \\ \sin(\hat{\theta}_k^-) & \cos(\hat{\theta}_k^-) \end{bmatrix} \begin{bmatrix} v_{xr} \\ v_{yr} \end{bmatrix}
$$

For linear states $[x, y, \theta, \dot{x}, \dot{y}, \dot{\theta}]^T$, the observation relationship $\mathbf{z}_k = \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k$ directly samples velocities and heading:

$$
\mathbf{H}_k = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & \frac{1}{\Delta t} & 0 & 0 \\
0 & 0 & 0 & 0 & \frac{1}{\Delta t} & 0
\end{bmatrix}
$$

The innovation (measurement residual) $\tilde{\mathbf{y}}_k$, innovation covariance $\mathbf{S}_k$, Kalman gain $\mathbf{K}_k$, and updated state $\hat{\mathbf{x}}_k$ are computed as:

$$
\tilde{\mathbf{y}}_k = \mathbf{z}_k - \mathbf{H}_k \hat{\mathbf{x}}_k^-
$$

:::{note}
When evaluating $\tilde{y}_{k, \theta} = \theta_{\text{imu}, k} - \hat{\theta}_k^-$, the heading error must be normalized to $[-\pi, \pi)$ via `atan2(sin(error), cos(error))` to eliminate modulo $2\pi$ wrap-around singularities.
:::

$$
\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k
$$

$$
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T \mathbf{S}_k^{-1}
$$

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \tilde{\mathbf{y}}_k
$$

$$
\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \quad \text{(Joseph Form)}
$$

---

### 3.4 Process and Measurement Covariance Tuning

The performance of the filter relies on the continuous-to-discrete derivation and physical parameter tuning of the process noise covariance $\mathbf{Q}_k$ and measurement noise covariance $\mathbf{R}_k$.

#### 3.4.1 Derivation and Tuning of Process Noise Covariance $\mathbf{Q}_k$

Process noise $\mathbf{w}_k$ models uncaptured physical effects: chassis structural flexing, deadwheel suspension spring oscillations, floor roughness, and micro-slipping.

Under the continuous white-noise acceleration model, the continuous disturbance power spectral density matrix $\mathbf{S}_w$ is:

$$
\mathbf{S}_w = \begin{bmatrix}
\sigma_{ax}^2 & 0 & 0 \\
0 & \sigma_{ay}^2 & 0 \\
0 & 0 & \sigma_{\alpha}^2
\end{bmatrix}
$$

where $\sigma_{ax}^2$ and $\sigma_{ay}^2$ ($(\text{m}/\text{s}^2)^2/\text{Hz}$) represent linear acceleration uncertainty, and $\sigma_{\alpha}^2$ ($(\text{rad}/\text{s}^2)^2/\text{Hz}$) represents angular acceleration uncertainty.

Evaluating the continuous-time integral for each $2 \times 2$ uncoupled kinematic block yields:

$$
\mathbf{Q}_k = \begin{bmatrix}
\mathbf{Q}_{xx} & \mathbf{0}_{2 \times 2} & \mathbf{0}_{2 \times 2} \\
\mathbf{0}_{2 \times 2} & \mathbf{Q}_{yy} & \mathbf{0}_{2 \times 2} \\
\mathbf{0}_{2 \times 2} & \mathbf{0}_{2 \times 2} & \mathbf{Q}_{\theta\theta}
\end{bmatrix}
$$

where each sub-block $\mathbf{Q}_{\xi\xi}$ for state $\xi \in \{x, y, \theta\}$ evaluates to:

$$
\mathbf{Q}_{\xi\xi} = \sigma_{\xi}^2 \begin{bmatrix}
\frac{\Delta t^3}{3} & \frac{\Delta t^2}{2} \\
\frac{\Delta t^2}{2} & \Delta t
\end{bmatrix}
$$

For $\Delta t = 0.02\text{ s}$:
- $\frac{\Delta t^3}{3} = \frac{0.000008}{3} \approx 2.67 \times 10^{-6}$
- $\frac{\Delta t^2}{2} = \frac{0.0004}{2} = 2.0 \times 10^{-4}$
- $\Delta t = 0.02$

**Tuning Rules for $\mathbf{Q}_k$:**
- If $\sigma_{ax}^2, \sigma_{ay}^2$ are tuned **too large**, $\mathbf{P}_k^-$ expands aggressively every cycle. The filter over-corrects on instantaneous, noisy sensor measurements, resulting in trajectory jitter.
- If $\sigma_{ax}^2, \sigma_{ay}^2$ are tuned **too small**, the filter over-relies on the constant-velocity kinematics, creating phase lag during sharp velocity reversals or sudden braking.

#### 3.4.2 Derivation and Tuning of Measurement Noise Covariance $\mathbf{R}_k$

Measurement noise covariance $\mathbf{R}_k \in \mathbb{R}^{6 \times 6}$ is diagonal, assuming sensor noise sources are statistically independent:

$$
\mathbf{R}_k = \begin{bmatrix}
\sigma_{vx,\text{enc}}^2 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma_{vy,\text{enc}}^2 & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma_{\omega,\text{enc}}^2 & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma_{\theta,\text{imu}}^2 & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma_{ax,\text{imu}}^2 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma_{ay,\text{imu}}^2
\end{bmatrix}
$$

The diagonal variance terms are determined through static characterization and dynamic operational analysis:

1. **Deadwheel Linear Velocities ($\sigma_{vx,\text{enc}}^2, \sigma_{vy,\text{enc}}^2$):**
   - In static conditions, optical encoder noise is negligible ($\sigma_{v,\text{static}}^2 \approx 10^{-6}\,(\text{m}/\text{s})^2$).
   - During dynamic locomotion, deadwheel ground scrub and vibrations increase velocity variance. We configure $\sigma_{vx,\text{enc}} = \sigma_{vy,\text{enc}} \approx 0.025\text{ m}/\text{s} \implies \sigma_{v,\text{enc}}^2 = 6.25 \times 10^{-4}\,(\text{m}/\text{s})^2$.

2. **Deadwheel Angular Rate vs. BNO085 Yaw ($\sigma_{\omega,\text{enc}}^2$ vs $\sigma_{\theta,\text{imu}}^2$):**
   - Deadwheel orientation estimation degrades when the robot rotates in place due to micro-slip along the omni-wheel roller contact points. Thus, the deadwheel angular rate variance is set moderately high: $\sigma_{\omega,\text{enc}}^2 = 0.04\,(\text{rad}/\text{s})^2$.
   - Conversely, the BNO085 Game Rotation Vector provides internal conical and sculling motion integration with zero magnetic interference. The short-term orientation variance is measured statically at $\sigma_{\theta,\text{imu}} \approx 0.005\text{ rad} \implies \sigma_{\theta,\text{imu}}^2 = 2.5 \times 10^{-5}\,\text{rad}^2$.
   - **Tuning Outcome**: Because $\sigma_{\theta,\text{imu}}^2 \ll \sigma_{\omega,\text{enc}}^2$, the Kalman gain $\mathbf{K}_k$ heavily weights the IMU Game Rotation Vector for heading, locking robot orientation to the BNO085 while relying on the deadwheels for planar translation.

3. **BNO085 Linear Acceleration ($\sigma_{ax,\text{imu}}^2, \sigma_{ay,\text{imu}}^2$):**
   - The linear acceleration reports (`BNO_REPORT_LINEAR_ACCEL`, `0x04`) isolate dynamic linear acceleration by subtracting gravity. However, high-frequency structural vibration from the drive motors propagates into the accelerometer.
   - We tune $\sigma_{a,\text{imu}} = 0.2\text{ m}/\text{s}^2 \implies \sigma_{a,\text{imu}}^2 = 0.04\,(\text{m}/\text{s})^2$. This allows acceleration to catch sudden collisions or quick jerks without disturbing steady-state velocity tracking.

| Covariance Parameter | Nominal Value | Physical Interpretation |
| :--- | :--- | :--- |
| $\sigma_{ax}^2, \sigma_{ay}^2$ | $0.50\,(\text{m}/\text{s}^2)^2/\text{Hz}$ | Process noise: models wheel bump dynamics and chassis vibration |
| $\sigma_{\alpha}^2$ | $0.80\,(\text{rad}/\text{s}^2)^2/\text{Hz}$ | Process noise: models rotational acceleration disturbances |
| $\sigma_{vx,\text{enc}}^2, \sigma_{vy,\text{enc}}^2$ | $6.25 \times 10^{-4}\,(\text{m}/\text{s})^2$ | Measurement noise: deadwheel linear translation variance |
| $\sigma_{\omega,\text{enc}}^2$ | $4.00 \times 10^{-2}\,(\text{rad}/\text{s})^2$ | Measurement noise: deadwheel angular rate slip variance |
| $\sigma_{\theta,\text{imu}}^2$ | $2.50 \times 10^{-5}\,\text{rad}^2$ | Measurement noise: high-precision BNO085 Game Rotation yaw |
| $\sigma_{ax,\text{imu}}^2, \sigma_{ay,\text{imu}}^2$ | $4.00 \times 10^{-2}\,(\text{m}/\text{s})^2$ | Measurement noise: accelerometer motor vibration variance |

---

## 4. Software & Hardware Interface

### 4.1 STM32 Firmware Driver Architecture

The firmware running on the STM32F411CEUx organizes low-level polling into distinct, modular drivers:

#### 4.1.1 Encoder Driver Module
Deadwheels are serviced in hardware by STM32 timer encoder peripherals configured in 4x counting mode (`TIM_ENCODERMODE_TI12`) with input capture digital glitch filtering:
- `encoder_init(&htim4, &htim3, &htim2)`: Starts all three encoder timer counters and initialises tracking variables.
- `encoder_update()`: Reads the instantaneous 16-bit register values `__HAL_TIM_GET_COUNTER()`, evaluates signed delta `delta = (int16_t)(raw - s_last_raw[i])` to handle automatic timer counter overflow/underflow, and accumulates cumulative ticks in `s_total[i]`.
- `encoder_get_count(uint8_t idx)`: Returns 32-bit cumulative count for deadwheel `idx` $\in \{0, 1, 2\}$.

#### 4.1.2 BNO085 SHTP IMU Driver Module
The IMU driver interfaces with the Hillcrest SH-2 Sensor Hub Transport Protocol (SHTP) across I2C3:
- `BNO_Port_Init(&s_bno_port)`: Hooks hardware-level I2C read, write, and delay functions to the platform-independent driver layer.
- `BNO_Port_Detect()`: Scans primary address `0x4A` (`0x94` HAL format) and alternate address `0x4B` (`0x96` HAL format) via `HAL_I2C_IsDeviceReady()`.
- `BNO085_Init(&s_bno, &s_bno_port)`: Establishes SHTP sequence tracking.
- `BNO085_EnableGameRotation(&s_bno, 20000)`: Sends an SH-2 `BNO_CMD_SET_FEATURE` command configuring report `0x08` for a 50 Hz reporting rate ($20000\,\mu\text{s}$).
- `BNO085_EnableLinearAccel(&s_bno, 20000)`: Configures linear acceleration report `0x04` at 50 Hz.
- `BNO085_Update(&s_bno)`: Unpacks 4-byte SHTP packet headers, verifies channel numbers, and decodes Q14-formatted quaternion reports ($q = \text{raw} / 16384.0$) and Q8-formatted linear acceleration vectors ($a = \text{raw} / 256.0\,\text{m}/\text{s}^2$).
- `BNO085_GetEulerYaw(&q)`: Converts decoded Game Rotation quaternion into Euler yaw angle in degrees:

```c
float BNO085_GetEulerYaw(const BNO_Quaternion *q)
{
    if (q == NULL) return 0.0f;
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    return atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.141592653589793f);
}
```

---

### 4.2 USB CDC Telemetry Protocol

The bidirectional communication link between the STM32F411 and Nvidia Jetson is implemented using the USB Communications Device Class (CDC) Virtual COM Port operating at Full Speed (12 Mbps PHY).

#### 4.2.1 Transmit & Receive Pipelines

1. **Transmission Pipeline (`CDC_Transmit_FS`)**:
   - Telemetry is assembled in `robot_loop()` every $20\text{ ms}$ (`TELEMETRY_INTERVAL_MS = 20`).
   - The buffer is checked to ensure the USB endpoint state is idle (`hcdc->TxState == 0`).
   - The packet is dispatched non-blockingly via `USBD_CDC_TransmitPacket()`.

2. **Reception Pipeline (`CDC_Receive_FS`)**:
   - Invoked asynchronously by the USB interrupt handler upon receiving host data packets.
   - Raw bytes are passed to `robot_process_rx(Buf, *Len)`, which buffers incoming line-delimited ASCII characters until `\r` or `\n` is detected.
   - The USB RX endpoint is immediately re-armed using `USBD_CDC_ReceivePacket(&hUsbDeviceFS)`.

#### 4.2.2 Telemetry Packet Schema

The 50 Hz JSON telemetry frame sent to the Jetson has the following schema:

```json
{
  "yaw": 42.15,
  "x": 0.05,
  "y": -0.02,
  "z": 0.01,
  "e1": 15420,
  "e2": -8920,
  "e3": 6451,
  "bno_ok": 1,
  "pwm": [250, 250, -250, -250]
}
```

| JSON Key | Type | Unit | Description |
| :--- | :--- | :--- | :--- |
| `yaw` | Float | Degrees ($^\circ$) | BNO085 Game Rotation Vector Yaw angle ($[-180.0, 180.0]$) |
| `x` | Float | $\text{m}/\text{s}^2$ | BNO085 Linear Acceleration along local X-axis (gravity removed) |
| `y` | Float | $\text{m}/\text{s}^2$ | BNO085 Linear Acceleration along local Y-axis (gravity removed) |
| `z` | Float | $\text{m}/\text{s}^2$ | BNO085 Linear Acceleration along local Z-axis (gravity removed) |
| `e1` | Integer | Ticks | Cumulative encoder tick count for Deadwheel 1 (TIM4) |
| `e2` | Integer | Ticks | Cumulative encoder tick count for Deadwheel 2 (TIM3) |
| `e3` | Integer | Ticks | Cumulative encoder tick count for Deadwheel 3 (TIM2) |
| `bno_ok` | Integer | Boolean | Health flag: `1` if BNO085 initialized and receiving reports, `0` otherwise |
| `pwm` | Array[4] | Integer | Current duty cycles applied to motor drivers M1..M4 ($-1000$ to $+1000$) |

### 4.3 ROS 2 Node Architecture (Nvidia Jetson)

On the Nvidia Jetson, a dedicated ROS 2 node interfaces with the virtual serial port `/dev/ttyACM0` (configured via udev rule as `/dev/karma_base`):

```{mermaid}
flowchart LR
    DEV["USB Serial: /dev/karma_base"] --> PARSE["serial_bridge Node"]
    PARSE -->|IMU Msg| IMU_PUB["/imu/data_raw"]
    PARSE -->|Wheel Tick Msg| ENC_PUB["/wheel_tick"]
    ENC_PUB --> BRAIN["smart_brain Node"]
    BRAIN -->|Raw Odom| EKF_NODE["ekf_node robot_localization"]
    IMU_PUB --> EKF_NODE
    EKF_NODE --> ODOM_PUB["/odometry/filtered"]
    EKF_NODE --> TF_PUB["TF: odom to base_link"]
```

```{image} assets/BNO085.jpeg
:alt: BNO085 IMU Sensor Module
:width: 360px
:align: center
```
*Figure 4: BNO085 9-DOF/6-DOF IMU sensor module with internal ARM Cortex-M0+ SH-2 sensor hub.*

#### 4.3.1 Yaw-to-Quaternion & Covariance Encoding

The IMU yaw is ingested over serial from the STM32 and converted to quaternion form on the ROS 2 host:

```python
def yaw_to_quaternion(yaw_degrees):
    yaw_rad = yaw_degrees * (math.pi / 180.0)
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qx, qy, qz, qw
```

To inform the Extended Kalman Filter of measurement validity, covariance values are explicitly configured:

```python
# sensor_msgs/Imu standard covariance tagging
imu_msg.angular_velocity_covariance[0] = -1.0      # Angular velocity unavailable
imu_msg.linear_acceleration_covariance[0] = -1.0   # Linear acceleration unavailable
imu_msg.orientation_covariance = [
    1e6, 0.0, 0.0,
    0.0, 1e6, 0.0,
    0.0, 0.0, 0.02   # High confidence in yaw measurement
]
```
* An element `[0] = -1.0` follows the ROS `sensor_msgs/Imu` standard denoting an unpopulated measurement axis, instructing `robot_localization` to ignore those fields.
* Large diagonal variance (`1e6`) on roll and pitch prevents the filter from falsely trusting the unmeasured planar axes, while small variance (`0.02`) designates yaw as the authoritative heading measurement.

---

### 4.4 `robot_localization` Extended Kalman Filter Configuration

The planar state estimator runs the official ROS 2 `robot_localization` package `ekf_node`, fusing raw wheel odometry and IMU yaw:

| Parameter | Configured Value | Functional Description / Tuning Rationale |
| :--- | :--- | :--- |
| `use_sim_time` | `true` (sim) / `false` (hw) | Synchronizes filter clock to `/clock` or system wall clock. |
| `frequency` | `100.0` | 100 Hz filter loop rate matching serial poll rates to eliminate latency. |
| `sensor_timeout` | `0.5` | 500 ms timeout window; isolates a dead channel if a sensor drops out. |
| `two_d_mode` | `true` | Constrains state estimate to 2D plane ($x, y, \theta$), eliminating 3D pitch/roll artifacts. |
| `publish_tf` | `true` | Broadcasts the dynamic `odom → base_link` transform directly from the EKF filter. |
| `publish_acceleration` | `false` | Disables acceleration publication to conserve compute and message bandwidth. |
| `odom_frame` | `odom` | Parent coordinate frame ID for the filtered odometry output. |
| `base_link_frame` | `base_link` | Child coordinate frame representing the robot base origin. |
| `world_frame` | `odom` | Fixed reference frame for the filter's world estimate. |
| `odom0` | `/odom_raw` | Input topic for wheel-derived odometry. |
| `odom0_config` | `[true, true, false, false, false, false, true, true, false, false, false, false, false, false, false]` | Trusts wheel odometry for $x, y, \dot{x}, \dot{y}$ position and planar velocities. Heading is excluded from wheel odometry to prevent slip contamination. |
| `odom0_queue_size` | `10` | Inbound message buffer depth. |
| `odom0_differential`| `false` | Treats input values as absolute odometric measurements. |
| `odom0_relative` | `true` | Normalizes readings relative to the startup position. |
| `imu0` | `/imu/data_raw` | Input topic for IMU orientation telemetry. |
| `imu0_config` | `[false, false, false, false, false, true, false, false, false, false, false, true, false, false, false]` | Trusts IMU exclusively for yaw ($\theta$) and angular yaw rate ($\dot{\theta}$). |
| `imu0_queue_size` | `10` | Inbound message buffer depth. |
| `imu0_differential`| `false` | Uses absolute orientation readings. |
| `imu0_relative` | `true` | Zero-references the initial IMU reading at system boot. |
| `imu0_remove_gravitational_acceleration` | `true` | Removes gravity vector from internal acceleration calculations. |
| `initial_estimate_covariance` | Diagonal matrix | Initial uncertainty matrix determining startup filter convergence rate. |
| `process_noise_covariance` | Diagonal matrix | Governs process uncertainty expansion and sensor weighting per time step. |

---

## 5. Known Issues & Troubleshooting

* **Issue: BNO085 fails to output sensor reports after boot sequence.**
  * **Root Cause:** BNO085 requires initial clearing of SHTP advertisement packets before feature commands are acknowledged.
  * **Firmware Fix:** The initialization sequence drains startup packets across 15 iterations with $10\text{ ms}$ delays. Additionally, a $250\text{ ms}$ periodic re-transmission watchdog triggers if `diag.rx_reports == 0`.

* **Issue: USB CDC transmission lockup during high serial traffic.**
  * **Root Cause:** Attempting to call `CDC_Transmit_FS()` while a previous packet transfer is still pending on the IN endpoint (`TxState != 0`).
  * **Firmware Fix:** `CDC_Transmit_FS()` explicitly checks `hcdc->TxState`. If busy, it returns `USBD_BUSY` without overwriting the buffer. Telemetry transmission skips the cycle and waits for the next 20 ms window.

* **Issue: Gyroscope Z-axis zero-rate drift over prolonged runtimes.**
  * **Root Cause:** Because the Game Rotation Vector does not use magnetometers to correct yaw drift, residual bias in the MEMS gyroscope causes gradual yaw drift ($1^\circ$ to $3^\circ$ per 10 minutes).
  * **System Level Fix:** Long-term absolute orientation correction is handled at the higher SLAM layer on the Jetson by fusing 2D LiDAR scans (`/scan`) through `slam_toolbox` / AMCL to publish an absolute `map -> odom` transform.

