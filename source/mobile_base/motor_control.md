# Drive Motors & Embedded Control

```{note}
This document specifies the drive motors, motor drivers, and embedded FreeRTOS firmware for the 4-wheel mobile base. It covers PWM generation, timer Auto-Reload Register (ARR) configuration, encoder decoding, communication with the host computer, and safety watchdogs.
```

## Overview & Objectives

### System Purpose & Computational Division
The KARMA mobile base utilizes an asymmetric compute topology comprising a high-performance companion computer (NVIDIA Jetson Orin) coupled to an embedded real-time microcontroller (STM32F411CEU6). The responsibilities are partitioned according to control loop determinism:

* **NVIDIA Jetson Orin (High-Level Autonomous Stack):** Executes Ubuntu 22.04 LTS and ROS 2 Humble/Iron. Handles perception, LiDAR SLAM, global/local path planning (Nav2), object manipulation kinematics, and behavior tree execution. Transmits target velocity vectors (`/cmd_vel`) and consumes fused spatial telemetry.
* **STM32F411CEU6 (Low-Level Real-Time Controller):** Provides hard real-time execution. Generates 20 kHz ultrasonic motor PWM signals, samples 3-channel quadrature encoder hardware counters, executes non-blocking SHTP packet parsing from the BNO085 IMU over I2C3, enforces an autonomous 150 ms hardware deadman safety watchdog, and provides bus fault recovery.

### Firmware Evolution: V0.0 Blueprint to V2.0 RTOS
The firmware underwent a five-stage evolutionary redesign:
1. **BaseCode V0.0 (Blueprint):** Initial peripheral allocation on STM32 CubeMX. Identified timer resource contention between 4-channel motor PWM and 3-channel encoder interfaces.
2. **BaseCode V0.1 (Encoder Bench):** Isolated bench-test harness verifying 4096 CPR X4 hardware decoding on TIM2, TIM3, and TIM4 using an ASCII CSV test stream (`E,e1,e2,e3\r\n`).
3. **BaseCode V1.0 (Bare-Metal Polling Superloop):** First integrated base code. Ran a monolithic `robot_loop()` superloop. Suffered from timing jitter (1–40 ms), silent USB packet drops (`USBD_BUSY`), USB transmit buffer overwrites (`JSONDecodeError`), and complete CPU lockups when I2C slave devices held the SDA line low.
4. **BaseCode V1.1 (Safety Clamped Superloop):** Retained the V1.0 bare-metal polling structure but added an electrical safety clamp (limiting motor PWM duty cycle to $\le 50\%$ of ARR) and GPIO pin mirroring on Motor 3 DIR (driving PB10/PB15 alongside PB2) to bypass a broken PCB trace.
5. **BaseCode V2.0 (Zero-Heap FreeRTOS + 2D State Matrix):** Re-architected into a preemptive Rate Monotonic Scheduling (RMS) real-time system with static memory allocation, a 2D State Matrix for IMU lifecycle/recovery, 9-clock I2C bit-banging, and semaphore-synchronized USB CDC flow control.

---

## Technical Specifications

### System Specifications

| Parameter | Operational Specification | Engineering Justification / Notes |
| :--- | :--- | :--- |
| **Microcontroller (MCU)** | STM32F411CEU6 (WeAct Black Pill) | ARM Cortex-M4F @ 96 MHz, 512 KB Flash, 128 KB SRAM, hardware FPU. |
| **Companion Compute** | NVIDIA Jetson Orin Nano / NX | 6-core/8-core ARM Cortex-A78AE, 20–40 TOPS AI compute, USB 3.2 Gen 2. |
| **RTOS Kernel** | FreeRTOS Kernel v202012.00 | Static allocation (`configSUPPORT_STATIC_ALLOCATION = 1`, zero heap). |
| **Tick Timebase** | 1000 Hz ($1.0\text{ ms}$) | Generated via TIM1 / SysTick exception handler. |
| **Actuation Power Bus** | 24.0 V DC Nominal (6S LiPo / LiFePO4) | Independent high-current power distribution board (PDB). |
| **Logic Power Bus** | 5.0 V DC @ 5.0 A (Buck) $\to$ 3.3 V LDO | Powers MCU, optoisolators, logic buffers, and BNO085. |
| **Motor Drivers** | 4x Cytron MD30C Rev 2.0 | Full NMOS H-Bridge, 5 V–30 V DC, 30 A continuous (80 A peak for 1 s). |
| **Drive Motors** | 4x High-Torque Brushed DC Motors | 24 V nominal, ~100–150 W continuous rating, geared output. |
| **PWM Frequency** | 20.0 kHz | Ultrasonic switching; eliminates audible coil whine and reduces current ripple. |
| **PWM Resolution** | 10-bit equivalent ($\text{ARR} = 999$, $\text{PSC} = 83$ @ 96 MHz) | Direct duty cycle scaling from 0 to 1000 ($0.0\%$ to $100.0\%$). |
| **Hardware Deadman** | 150 ms Timeout | Dedicated check inside 100 Hz `MotorTask`; kills PWM on communication loss. |
| **Inertial Sensor** | CEVA / Hillcrest Labs BNO085 | 9-DOF SiP with Cortex-M0+ running SH-2 sensor fusion firmware. |
| **IMU Bus** | I2C3 @ 400 kHz Fast-Mode | PA8 (SCL) and PB8 (SDA) with automatic 9-clock recovery. |
| **Tracking Geometry** | Radius $R = 35.0\text{ mm}$, Track $W = 150.0\text{ mm}$ | 3-wheel ground-sprung dead-wheel omni assembly. |
| **Tracking Resolution** | 4096 Counts/Rev (1024 CPR Optical, X4 Decode) | $k_s \approx 5.371 \times 10^{-5}\text{ m/tick}$ ($0.0537\text{ mm/count}$). |
| **Telemetry Transport** | USB 2.0 Full Speed (12 Mbps PHY) | Virtual COM Port (CDC), 115200 baud nominal framing, 50 Hz rate. |

### Actuator & Motor Driver Specifications

| System Component | Specification Parameter | Concrete Value / Engineering Threshold |
| :--- | :--- | :--- |
| **Drive Motors (4x)** | Motor Type | Permanent Magnet Brushed DC Motors (PMDC) with integrated planetary reduction. |
| | Rated Nominal Voltage | 24.0 V DC (Operating envelope: 18.0 V to 28.0 V DC). |
| | Rated Mechanical Power | 100 W to 150 W per channel continuous output. |
| | Stall Current per Motor | 25.0 A to 35.0 A @ 24.0 V DC. |
| | No-Load Current | $\le 1.2\text{ A}$ per channel @ 24.0 V DC. |
| | Mechanical Coupling | Direct axle coupling to 4 independent drive wheels on KARMA chassis. |
| **Motor Drivers (Current)** | Driver Model | Cytron MD30C Rev 2.0 (Single-Channel H-Bridge). |
| | Power MOSFET Architecture | Full N-Channel MOSFET H-Bridge (ultra-low $R_{DS(on)}$ for passive thermal dissipation). |
| | Operating Voltage Range | 5.0 V DC to 30.0 V DC. |
| | Continuous Output Current | 30.0 A continuous (ambient $25^\circ\text{C}$, natural convection, no external fan). |
| | Peak Output Current | 80.0 A for $\le 1.0\text{ second}$ (inrush / high stall torque transient handling). |
| | Logic Input Compatibility | 3.3 V and 5.0 V CMOS/TTL compatible (directly driven by STM32 3.3 V I/O). |
| | Control Signaling Topology | Sign-Magnitude PWM: PWM pin sets duty cycle (speed); DIR pin sets polarity. |
| | Switching Frequency Ceiling | Up to 20.0 kHz ultrasonic operation. |
| | Dynamic Braking Mode | Low-side MOSFET regenerative braking when PWM $= 0$ or decelerating. |
| | Form Factor & Dimensions | $111.0\text{ mm} \times 48.0\text{ mm}$ per unit (bulk footprint). |
| **Motor Drivers (Next-Gen)** | Planned Driver Model | **Pololu Single-Channel 13A Motor Driver (e.g. G2 High-Power 24v13)**. |
| | Validation Status | **Bench-tested & verified; excellent dynamic response and thermal performance.** |
| | Operating Voltage Range | 6.5 V DC to 40.0 V DC (fully rated for 24.0 V nominal bus). |
| | Continuous Output Current | 13.0 A continuous per channel (matches PMDC motor continuous draw of 5–8 A). |
| | Peak Output Current | ~30.0 A transient peak handling. |
| | Form Factor & Dimensions | **$33.0\text{ mm} \times 20.3\text{ mm}$** (~87% footprint reduction vs. MD30C). |
| | Control Signaling Topology | Identical Sign-Magnitude PWM & DIR interface (100% firmware backward-compatible). |
| **Safety Protections** | Duty Cycle Safety Ceiling | Software clamp (`MOTOR_MAX_PWM_LIMIT`) restricting duty cycle to $\le 50\%$ ARR. |
| | Deadman Watchdog Cutoff | 150 ms timeout in 100 Hz `MotorTask`; commands all CCR registers to 0. |
| | Direction Pin Redundancy | Motor 3 DIR driven on PB2, mirrored to PB10 and PB15 to bypass PCB trace fault. |

---

## Design & Architecture

### FreeRTOS Real-Time Multi-Tasking Architecture
BaseCode V2.0 operates as a preemptive, multi-tasking system using Rate Monotonic Scheduling (RMS). Under RMS, tasks with higher execution frequencies receive higher execution priorities, ensuring optimal CPU scheduling without priority inversion.

```{mermaid}
flowchart TD
    subgraph Jetson_Orin ["Companion Host: NVIDIA Jetson Orin"]
        HostROS["ROS 2 Navigation Stack / karma_base_bridge"]
    end

    subgraph Hardware_ISRs ["Hardware Interrupt Layer"]
        USBRxISR["USB OTG FS RX Interrupt"]
        USBTxISR["USB OTG FS TX Complete Interrupt"]
    end

    subgraph RTOS_Kernel ["FreeRTOS Preemptive Kernel - Zero Heap"]
        q_rx["q_usb_rx Queue: 256 Bytes"]
        sem_tx["xUsbTxSemaphore: Binary Semaphore"]
        q_cmd["q_motor_cmd Queue: 4 Packets"]
        q_sensor["q_sensor_data Queue: 4 Packets"]
        mtx_state["xRobotStateMutex: Mutex"]

        TaskMotor["MotorTask: Priority 4, 100 Hz<br/>150 ms Deadman Watchdog<br/>TIM5 Compare and GPIOB Direction"]
        TaskAcq["AcquisitionTask: Priority 3, 50 Hz<br/>2D State Matrix BNO085 Driver<br/>16-Bit Rollover Hardware Encoders"]
        TaskEst["EstimationTask: Priority 2, 50 Hz<br/>Dead-Reckoning Integration<br/>Updates Protected Robot State"]
        TaskComm["CommunicationTask: Priority 1<br/>jsmn Zero-Alloc In-Place Parser<br/>50 Hz Formatted JSON Telemetry"]
        TimerLog["Logger Timer: Daemon 500 ms<br/>Ring Buffer Diagnostics Flush"]
    end

    HostROS -->|JSON Command Frames| USBRxISR
    USBRxISR -->|Push Raw Bytes| q_rx
    q_rx --> TaskComm
    TaskComm -->|motor_cmd_t| q_cmd
    q_cmd --> TaskMotor

    TaskAcq -->|sensor_data_t| q_sensor
    q_sensor --> TaskEst
    TaskEst --> mtx_state
    mtx_state --> TaskComm

    TaskComm -->|CDC_Transmit_FS| sem_tx
    USBTxISR -->|xSemaphoreGiveFromISR| sem_tx
    sem_tx -->|USB CDC Telemetry Frame| HostROS
```

### Static Zero-Heap Memory Allocation
To eliminate the risk of dynamic heap exhaustion or fragmentation during continuous multi-hour competition runs, BaseCode V2.0 operates with dynamic allocation disabled:

* `configSUPPORT_STATIC_ALLOCATION` is set to `1` in `FreeRTOSConfig.h`.
* `configSUPPORT_DYNAMIC_ALLOCATION` is set to `0`.
* The FreeRTOS memory manager `heap_x.c` is excluded from the build.
* All tasks, task stacks (`StackType_t`), queues, semaphores, and mutexes are declared as static globals:

```{code-block} c
/* Static memory allocations from tasks/task_manager.c */
static StaticTask_t  s_motor_tcb;
static StackType_t   s_motor_stack[256];

static StaticQueue_t s_motor_cmd_qcb;
static uint8_t       s_motor_cmd_storage[Q_MOTOR_CMD_LEN * sizeof(motor_cmd_t)];

QueueHandle_t q_motor_cmd = NULL;

void task_manager_init(void) {
    q_motor_cmd = xQueueCreateStatic(
        Q_MOTOR_CMD_LEN,
        sizeof(motor_cmd_t),
        s_motor_cmd_storage,
        &s_motor_cmd_qcb
    );

    xTaskCreateStatic(
        motor_task_fn,
        "MotorTask",
        256,
        NULL,
        PRIORITY_MOTOR_TASK,
        s_motor_stack,
        &s_motor_tcb
    );
}
```

### 2D State Matrix for IMU Lifecycle & Fault Recovery (`imu_sm.c`)

#### State Matrix Theory & Motivation
In BaseCode V1.0, BNO085 initialization, packet processing, and error handling were embedded in a linear superloop. If an I2C transaction stalled (e.g., due to inductive noise from motor switching holding the SDA line low), `HAL_I2C_Master_Receive()` blocked the CPU, stopping encoder tracking, motor control, and telemetry.

BaseCode V2.0 replaces this with a formal **2D State Matrix (Finite State Machine Transition Table)**. The state machine maps every combination of current state ($S$) and input event ($E$) to a deterministic tuple containing the target state and an action function pointer:

```{math}
\mathcal{T}: \mathcal{S} \times \mathcal{E} \longrightarrow \left( \mathcal{S}_{\text{next}}, \, f_{\text{action}} \right)
```

Where:
* States $\mathcal{S} = \{\text{UNINIT}, \text{DETECT}, \text{INIT\_DRIVER}, \text{CONFIG\_REPORTS}, \text{RUNNING}, \text{BUS\_RECOVERY}, \text{ERROR}\}$ ($S = 7$).
* Events $\mathcal{E} = \{\text{EVT\_STEP}, \text{EVT\_SUCCESS}, \text{EVT\_FAIL}, \text{EVT\_BUS\_ERROR}\}$ ($E = 4$).

```{mermaid}
stateDiagram-v2
    [*] --> UNINIT
    UNINIT --> DETECT : EVT_STEP / action_do_detect
    DETECT --> INIT_DRIVER : EVT_SUCCESS / action_do_init_driver
    DETECT --> BUS_RECOVERY : EVT_FAIL or EVT_BUS_ERROR
    INIT_DRIVER --> CONFIG_REPORTS : EVT_SUCCESS / action_do_config_reports
    INIT_DRIVER --> BUS_RECOVERY : EVT_FAIL or EVT_BUS_ERROR
    CONFIG_REPORTS --> RUNNING : EVT_SUCCESS / action_do_run
    CONFIG_REPORTS --> BUS_RECOVERY : EVT_FAIL or EVT_BUS_ERROR
    RUNNING --> RUNNING : EVT_STEP or EVT_SUCCESS
    RUNNING --> BUS_RECOVERY : EVT_BUS_ERROR / action_do_bus_recovery
    BUS_RECOVERY --> DETECT : EVT_SUCCESS / action_do_detect
    BUS_RECOVERY --> ERROR : EVT_FAIL / action_do_error
    ERROR --> BUS_RECOVERY : EVT_STEP / action_do_bus_recovery
```

#### 2D Transition Table Code Implementation
The matrix lookup executes in $O(1)$ constant time with zero conditional branching:

```{code-block} c
/* Transition cell definition from state_machines/imu_sm.h */
typedef struct {
    imu_state_t     next_state;
    imu_action_fn_t action;
} imu_transition_t;

/* 2D Transition Matrix from state_machines/imu_sm.c */
static const imu_transition_t s_transition_table[IMU_STATE_COUNT][IMU_EVT_COUNT] = {
    [IMU_STATE_UNINIT] = {
        [IMU_EVT_STEP]      = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_DETECT] = {
        [IMU_EVT_STEP]      = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_INIT_DRIVER,   action_do_init_driver },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_INIT_DRIVER] = {
        [IMU_EVT_STEP]      = { IMU_STATE_INIT_DRIVER,   action_do_init_driver },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_CONFIG_REPORTS,action_do_config_reports },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_CONFIG_REPORTS] = {
        [IMU_EVT_STEP]      = { IMU_STATE_CONFIG_REPORTS,action_do_config_reports },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_RUNNING,       action_do_run },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_RUNNING] = {
        [IMU_EVT_STEP]      = { IMU_STATE_RUNNING,       action_do_run },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_RUNNING,       action_none },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_BUS_RECOVERY] = {
        [IMU_EVT_STEP]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_FAIL]      = { IMU_STATE_ERROR,         action_do_error },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_ERROR,         action_do_error }
    },
    [IMU_STATE_ERROR] = {
        [IMU_EVT_STEP]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_FAIL]      = { IMU_STATE_ERROR,         action_none },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_ERROR,         action_none }
    }
};

void imu_sm_dispatch(imu_event_t evt) {
    if (evt >= IMU_EVT_COUNT || s_current_state >= IMU_STATE_COUNT) return;

    imu_transition_t transition = s_transition_table[s_current_state][evt];
    if (transition.next_state < IMU_STATE_COUNT) {
        s_current_state = transition.next_state;
    }
    if (transition.action != NULL && transition.action != action_none) {
        transition.action();
    }
}
```

#### Automatic 9-Clock I2C Recovery Sequence
When four consecutive communication errors occur in `action_do_run`, an `IMU_EVT_BUS_ERROR` event transitions the system to `IMU_STATE_BUS_RECOVERY`, executing `drivers/i2c_recovery.c`:

1. **Peripheral Disablement:** Disables hardware peripheral I2C3 and configures PA8 (SCL) and PB8 (SDA) as Open-Drain GPIO outputs with internal pull-ups.
2. **Shift Register Clocking:** Toggles PA8 (SCL) through 9 clock cycles ($5\,\mu\text{s}$ low, $5\,\mu\text{s}$ high, generating a $100\text{ kHz}$ recovery clock). This clocks the slave's internal shift register until it releases the SDA data line.
3. **STOP Condition Generation:** Generates a manual STOP condition by driving SDA low, pulling SCL high, and then releasing SDA high while SCL remains high.
4. **Peripheral Restoration:** Reconfigures PA8 and PB8 to alternate function mode (`GPIO_AF4_I2C3` / `GPIO_AF9_I2C3`) and reinitializes hardware I2C3.

### Timer Auto-Reload Register (ARR) & PWM Mathematics

#### Timer APB Clock Architecture
On the STM32F411CEU6, the internal PLL is configured as follows:
* Crystal Input ($f_{\text{HSE}}$): $25.0\text{ MHz}$ (or $16.0\text{ MHz}$ internal HSI).
* PLL Configuration: $\text{PLLM} = 15$, $\text{PLLN} = 144$, $\text{PLLP} = 2$, yielding system clock $f_{\text{SYSCLK}} = 96.0\text{ MHz}$.
* AHB Prescaler = 1 ($f_{\text{HCLK}} = 96.0\text{ MHz}$).
* APB1 Prescaler = 1 ($f_{\text{PCLK1}} = 96.0\text{ MHz}$).
* When the APB prescaler is 1, the timer multiplier is 1; thus, the timer clock feeding TIM5 is:
  ```{math}
  f_{\text{TIM5\_CLK}} = 96.0\text{ MHz}
  ```

#### PWM Frequency Derivation
The PWM output frequency ($f_{\text{PWM}}$) is governed by the timer clock, Prescaler (PSC), and Auto-Reload Register (ARR):

```{math}
f_{\text{PWM}} = \frac{f_{\text{TIM}}}{(\text{PSC} + 1) \times (\text{ARR} + 1)}
```

To achieve an ultrasonic switching frequency of $f_{\text{PWM}} = 20.0\text{ kHz}$ (which prevents audible switching whine and reduces current ripple through the motor windings):

Given Prescaler $\text{PSC} = 83$ (dividing by 84):
```{math}
(\text{ARR} + 1) = \frac{96 \times 10^6\text{ Hz}}{84 \times 20 \times 10^3\text{ Hz}} = \frac{96 \times 10^6}{1.68 \times 10^6} \approx 57.14
```

Alternatively, utilizing the standard CubeMX configuration with $\text{PSC} = 1$ (divide by 2, timer tick $= 48.0\text{ MHz}$):
```{math}
(\text{ARR} + 1) = \frac{48.0\times 10^6\text{ Hz}}{20.0\times 10^3\text{ Hz}} = 2400 \implies \mathbf{\text{ARR} = 2399}
```

In the active BaseCode V1.0/V2.0 implementation, a 1 kHz base timebase was tested with $\text{PSC} = 1$ and $\text{Period (ARR)} = 999$, scaling the compare match linearly from $0$ to $999$:

```{math}
\text{Duty Cycle (\%)} = \left( \frac{\text{CCR}}{\text{ARR} + 1} \right) \times 100\%
```

#### Motor Setpoint to Compare Register (CCR) Mapping
Commanded speed setpoints are received from the host as normalized signed integers in the range $[-1000, +1000]$ (corresponding to $-100.0\%$ to $+100.0\%$ duty cycle).

The translation in `motor.c` / `motor_driver.c` executes via 64-bit unsigned integer arithmetic:

```{code-block} c
void motor_set(uint8_t motor_id, int16_t speed) {
    if (motor_id >= NUM_MOTORS) return;

    /* 1. Software ceiling clamp (V1.1 safety patch: 50% max limit) */
    if (speed >  MOTOR_MAX_PWM_LIMIT) speed =  MOTOR_MAX_PWM_LIMIT;
    if (speed < -MOTOR_MAX_PWM_LIMIT) speed = -MOTOR_MAX_PWM_LIMIT;

    /* 2. Direction GPIO evaluation */
    if (speed > 0) {
        HAL_GPIO_WritePin(s_motors[motor_id].dir_port, s_motors[motor_id].dir_pin, GPIO_PIN_SET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(s_motors[motor_id].dir_port, s_motors[motor_id].dir_pin, GPIO_PIN_RESET);
    }

    /* 3. Magnitude and ARR duty cycle calculation */
    uint32_t mag = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(s_htim); // Reads active ARR

    /* Scaled integer mapping: CCR = (mag * period) / 1000 */
    uint32_t duty = (uint32_t)(((uint64_t)mag * (uint64_t)period) / 1000ULL);

    /* 4. Hardware overstress clamp: ensure duty cycle never exceeds 50% ARR */
    if (duty > (period / 2)) {
        duty = period / 2;
    }

    /* 5. Direct write to Timer Output Compare Register */
    __HAL_TIM_SET_COMPARE(s_htim, s_motors[motor_id].tim_channel, duty);
}
```

### Actuator Drive Mechanics & MD30C Driver Operation

#### Physical Motor Characteristics & Electrical Topology
The KARMA base utilizes four high-power permanent magnet brushed DC (PMDC) drive motors coupled to planetary gear reduction boxes to drive the chassis:
* **Operating Supply:** Direct 24.0 V DC feed from the primary power distribution board (PDB).
* **Power & Current Envelope:** Each motor is rated for 100 W to 150 W continuous mechanical output, drawing ~1.2 A at no-load and up to 35.0 A peak under heavy acceleration or stall conditions.
* **Chassis Actuation Mapping:** Four independently driven wheels configured in a differential / omnidirectional drive setup:
  * Motor 1 ($M_1$): Front-Left drive assembly.
  * Motor 2 ($M_2$): Front-Right drive assembly.
  * Motor 3 ($M_3$): Rear-Left drive assembly.
  * Motor 4 ($M_4$): Rear-Right drive assembly.

#### Cytron MD30C H-Bridge Architecture
Each motor is driven by a dedicated Cytron MD30C Rev 2.0 single-channel motor driver:
* **Full NMOS Solid-State H-Bridge:** Built with discrete N-channel power MOSFETs that achieve ultra-low $R_{DS(on)}$ resistance, eliminating the severe $2\text{–}4\text{ V}$ voltage drops and massive thermal dissipation characteristic of legacy bipolar drivers (e.g. L298N). It supports 30 A continuous and 80 A peak current for up to 1 second.
* **Sign-Magnitude Control Interface:**
  * **PWM Input Pin:** Receives the 20 kHz PWM square wave from STM32 TIM5 (PA0..PA3). Duty cycle controls the effective average DC voltage applied across the motor terminals:
    ```{math}
    V_{\text{motor}} = D \times V_{\text{bus}} = \left( \frac{\text{CCR}}{\text{ARR}} \right) \times 24.0\text{ V}
    ```
  * **DIR Input Pin:** Driven by standard STM32 GPIO push-pull outputs (PB0, PB1, PB2, PB12). A digital HIGH (`GPIO_PIN_SET`) directs current from Terminal A to Terminal B; a digital LOW (`GPIO_PIN_RESET`) directs current from Terminal B to Terminal A.
* **Optocoupler & Logic Compatibility:** The MD30C features onboard optoisolated inputs and logic level shifters supporting both 3.3 V and 5.0 V logic. The 3.3 V outputs from the STM32F411 drive the driver inputs directly with zero external level shifting.
* **Dynamic Braking vs Coasting:** When PWM is driven to $0\%$ or commanded speed drops, the MD30C actively conducts through both low-side MOSFETs, shorting the motor terminals. This dissipates the kinetic energy via the motor's internal armature resistance and back-EMF, achieving fast dynamic braking rather than uncontrolled coasting.
* **Opposite-Side Chassis Inversion:** Because drive motors on the left and right sides are mounted in 180° opposing physical orientations, identical forward rotation requires opposite electrical polarity. Firmware or wiring sense compensates to ensure all wheels drive forward under positive speed commands.

#### Actuation Safety & Fault Protections
1. **Software Duty Cycle Ceiling (`MOTOR_MAX_PWM_LIMIT`):** Clamps PWM compare registers to a maximum of $50\%$ of ARR, preventing excessive current draw during development and bench testing.
2. **150 ms Deadman Safety Cutoff:** Evaluated every 10 ms inside `MotorTask`. If communication with the Jetson freezes or drops for longer than 150 ms, all four PWM outputs are immediately driven to $0\%$, halting the robot.
3. **Hardware Direction Pin Mirroring:** For Motor 3, the firmware simultaneously drives PB2 (primary), PB10 (adjacent pin), and PB15 (legacy pin) to ensure electrical continuity across a damaged PCB trace.

#### Planned Driver Miniaturization: Pololu Single-Channel 13A Drivers
While the current revisions rely on the robust Cytron MD30C, future iterations of the KARMA base will transition to **Pololu Single-Channel 13A Motor Drivers** (e.g., G2 High-Power 24v13):
* **Footprint Optimization:** The Cytron MD30C measures $111.0 \times 48.0\text{ mm}$ ($53.3\text{ cm}^2$ per board, totaling $>213\text{ cm}^2$ for 4 channels). The Pololu 13A driver occupies only $33.0 \times 20.3\text{ mm}$ ($6.7\text{ cm}^2$ per board), achieving an **~87% reduction in driver footprint** and major chassis weight savings.
* **Current & Thermal Sufficiency:** The 24 V PMDC motors draw 5 A–8 A under continuous load and up to 25 A–35 A stall. The Pololu 13A continuous / 30 A peak rating fits the motor operational envelope with low thermal dissipation.
* **Empirical Validation:** Bench testing of the Pololu 13A drivers has already been conducted under continuous load cycles, confirming stable thermal dissipation, linear PWM duty cycle response, and high reliability.
* **Firmware Compatibility:** The Pololu module operates on the identical Sign-Magnitude PWM and DIR signaling paradigm (logic-level PWM + DIR), enabling a direct hardware swap with **zero firmware code modifications**.

### Sensor Acquisition Mechanisms

#### Optical Quadrature Encoders (TIM4, TIM3, TIM2)
The three tracking dead wheels use optical quadrature encoders with channels A and B routed to timer input pins TI1 and TI2:
* **Decoding Mode:** Timers configured in `TIM_ENCODERMODE_TI12` (counts on both rising and falling edges of TI1 and TI2, providing 4 counts per physical line pair).
* **Physical Lines:** 1024 CPR optical disk $\to$ $1024 \times 4 = 4096\text{ counts/revolution}$.
* **Branchless Rollover Arithmetic:** Timer registers are 16-bit (`uint16_t`, 0 to 65535). Delta tracking between consecutive sampling periods uses two's complement unsigned integer underflow/overflow:

```{code-block} c
uint16_t current_raw = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
int16_t  delta_ticks = (int16_t)(current_raw - previous_raw);
previous_raw = current_raw;
accumulated_ticks += (int32_t)delta_ticks;
```

*Proof of Correctness across Boundaries:*
* *Forward Boundary Crossing ($65535 \to 0$):*
  ```{math}
  \text{delta} = (int16\_t)(0\text{x}0000 - 0\text{x}FFFF) = (int16\_t)(0\text{x}0001) = +1\text{ tick}
  ```
* *Reverse Boundary Crossing ($0 \to 65535$):*
  ```{math}
  \text{delta} = (int16\_t)(0\text{x}FFFF - 0\text{x}0000) = (int16\_t)(0\text{x}FFFF) = -1\text{ tick}
  ```

#### BNO085 9-DOF IMU Sensor Hub (SHTP over I2C3)
The BNO085 integrates a triaxial accelerometer, triaxial gyroscope, and triaxial magnetometer with an internal 32-bit ARM Cortex-M0+ executing Hillcrest SH-2 sensor fusion firmware.

* **Physical Bus:** I2C3 operating at 400 kHz Fast-Mode. 7-bit slave address: `0x4A` (default) or `0x4B` (alternate).
* **Sensor Hub Transport Protocol (SHTP):** Communication frames consist of a 4-byte header followed by payload:
  * Byte 0: `Length LSB` (lower 8 bits of total frame length, including header).
  * Byte 1: `Length MSB` (upper 7 bits; MSB indicates continuation).
  * Byte 2: `Channel Number` (0: Command, 1: Executable, 2: Control/SetFeature, 3: Input Sensor Reports).
  * Byte 3: `Sequence Number` (monotonically incrementing rollover counter).
* **Sensor Report Configuration:**
  1. **Game Rotation Vector (`0x08`):** Configured for 50 Hz ($20,000\,\mu\text{s}$ interval). Provides 4-element orientation quaternions $(q_w, q_x, q_y, q_z)$.
     ```{important}
     The Game Rotation Vector fuses triaxial gyroscope and accelerometer readings while explicitly excluding the magnetometer. This prevents magnetic distortion caused by steel arena frames, structural base members, and the magnetic fields generated by the 30 A motor lines.
     ```
  2. **Linear Acceleration (`0x04`):** Configured for 50 Hz ($20,000\,\mu\text{s}$ interval). Provides triaxial acceleration with gravity mathematically subtracted $(a_x, a_y, a_z)$ in $\text{m/s}^2$.
* **Euler Yaw Conversion:** Quaternion components are converted to planar heading:
  ```{math}
  \text{Yaw} = \text{atan2}\left(2(q_w q_z + q_x q_y), \, 1 - 2(q_y^2 + q_z^2)\right)
  ```

---

## Software/Hardware Interface

### Master Hardware Pin & Peripheral Configuration

| Hardware Module | Peripheral | STM32 Pin | Mode / AF | Destination / Connected Signal | Voltage Level |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Actuation | TIM5_CH1 | PA0 | AF2 (TIM5) | MD30C Motor 1 PWM Input (20 kHz) | 3.3 V CMOS |
| Actuation | TIM5_CH2 | PA1 | AF2 (TIM5) | MD30C Motor 2 PWM Input (20 kHz) | 3.3 V CMOS |
| Actuation | TIM5_CH3 | PA2 | AF2 (TIM5) | MD30C Motor 3 PWM Input (20 kHz) | 3.3 V CMOS |
| Actuation | TIM5_CH4 | PA3 | AF2 (TIM5) | MD30C Motor 4 PWM Input (20 kHz) | 3.3 V CMOS |
| Actuation | GPIO Output | PB0 | Output Push-Pull | MD30C Motor 1 Direction Input | 3.3 V CMOS |
| Actuation | GPIO Output | PB1 | Output Push-Pull | MD30C Motor 2 Direction Input | 3.3 V CMOS |
| Actuation | GPIO Output | PB2 | Output Push-Pull | MD30C Motor 3 Direction Input (Primary) | 3.3 V CMOS |
| Actuation | GPIO Output | PB10 | Output Push-Pull | MD30C Motor 3 Direction Input (V1.1 Redundant A) | 3.3 V CMOS |
| Actuation | GPIO Output | PB15 | Output Push-Pull | MD30C Motor 3 Direction Input (V1.1 Redundant B) | 3.3 V CMOS |
| Actuation | GPIO Output | PB12 | Output Push-Pull | MD30C Motor 4 Direction Input | 3.3 V CMOS |
| Odometry | TIM4_CH1 | PB6 | AF2 (TIM4) | Dead-Wheel Encoder 1 Channel A | 3.3 V / 5 V FT |
| Odometry | TIM4_CH2 | PB7 | AF2 (TIM4) | Dead-Wheel Encoder 1 Channel B | 3.3 V / 5 V FT |
| Odometry | TIM3_CH1 | PB4 | AF2 (TIM3) | Dead-Wheel Encoder 2 Channel A | 3.3 V / 5 V FT |
| Odometry | TIM3_CH2 | PB5 | AF2 (TIM3) | Dead-Wheel Encoder 2 Channel B | 3.3 V / 5 V FT |
| Odometry | TIM2_CH1 | PA15 | AF1 (TIM2) | Dead-Wheel Encoder 3 Channel B | 3.3 V / 5 V FT |
| Odometry | TIM2_CH2 | PB3 | AF1 (TIM2) | Dead-Wheel Encoder 3 Channel A | 3.3 V / 5 V FT |
| Sensors | I2C3_SCL | PA8 | AF4 / GPIO | BNO085 SCL (400 kHz / 9-Clock Recovery) | 3.3 V OD (PU) |
| Sensors | I2C3_SDA | PB8 | AF9 / GPIO | BNO085 SDA (400 kHz / Manual STOP) | 3.3 V OD (PU) |
| USB Port | OTG_FS_DM | PA11 | AF10 (USB_FS) | Jetson Orin Full Speed USB D- | USB PHY |
| USB Port | OTG_FS_DP | PA12 | AF10 (USB_FS) | Jetson Orin Full Speed USB D+ | USB PHY |

### Host-Microcontroller Communication Pipeline

#### Inbound Command Pipeline (Jetson $\to$ STM32)
1. **Interrupt Buffering:** Incoming USB CDC bytes trigger the USB OTG interrupt handler. Bytes are pushed directly into static queue `q_usb_rx` inside the ISR with execution time under $1\,\mu\text{s}$, avoiding string parsing in interrupt context.
2. **In-Place Tokenization:** `CommunicationTask` unblocks on `q_usb_rx`, buffering bytes into a line buffer until `\r` or `\n` is encountered.
3. **Zero-Allocation JSON Parsing:** The string is parsed in-place using `jsmn.h` (a zero-heap tokenizer), eliminating dynamic memory allocation:
   * **Array Command:** `{"m": [250, 250, -250, -250]}`
   * **Individual Channel Command:** `{"m1": 250, "m2": 250, "m3": -250, "m4": -250}`
   * **Emergency Stop:** `{"cmd": "stop"}`
4. **Queue Dispatch & Watchdog Reset:** The parsed speeds are packed into `motor_cmd_t` and dispatched to `q_motor_cmd`. The arrival timestamp `s_last_cmd_tick` is updated, resetting the 150 ms deadman safety timer.

#### Outbound Telemetry Pipeline (STM32 $\to$ Jetson)
1. **State Snapshot Copy:** Every 20 ms (50 Hz), `CommunicationTask` reads the global `robot_state_t` snapshot under `xRobotStateMutex` protection.
2. **Buffer Serialization:** Serializes the state into ASCII JSON within static buffer `s_tx_buf`:
   ```json
   {"yaw": -45.12, "x": 0.02, "y": -0.03, "z": 9.81, "e1": 12450, "e2": 12438, "e3": 15, "bno_ok": 1, "pwm": [250, 250, -250, -250]}
   ```
3. **Semaphore Flow Control:** Before initiating transmission, `CommunicationTask` takes binary semaphore `xUsbTxSemaphore`.
4. **Hardware Non-Blocking Transmission:** `CDC_Transmit_FS()` passes the buffer pointer to the USB peripheral.
5. **Interrupt Completion:** When the hardware finishes transmitting the packet across the physical cable, the USB interrupt triggers `CDC_TransmitCplt_FS()`, which calls `xSemaphoreGiveFromISR(xUsbTxSemaphore)`. This prevents `USBD_BUSY` packet drops and eliminates buffer overwrite race conditions.

### ROS 2 Interface Specifications (`karma_base_bridge`)

| ROS 2 Topic | Message Type | Direction | QoS Profile | Description / Unit |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscription | Reliable, Depth=10 | Commanded planar velocity vector ($v_x, v_y$ in $\text{m/s}$, $\omega_z$ in $\text{rad/s}$). |
| `/odom` | `nav_msgs/msg/Odometry` | Publication | Best Effort, Depth=10 | 50 Hz dead-reckoning odometry position ($x, y$ in $\text{m}$), orientation quaternion, and linear/angular velocity. |
| `/imu/data` | `sensor_msgs/msg/Imu` | Publication | Best Effort, Depth=10 | 50 Hz BNO085 orientation quaternion ($q_w, q_x, q_y, q_z$) and linear acceleration ($a_x, a_y, a_z$ in $\text{m/s}^2$). |
| `/joint_states` | `sensor_msgs/msg/JointState` | Publication | Best Effort, Depth=10 | Unwrapped cumulative angular positions and angular velocities of tracking wheels. |

---

## Future Work / Known Limitations

### Target Architecture: Multi-Layer Hardware Abstraction Layer (HAL)
To make sensor drivers hardware-agnostic, future revisions will implement a layered driver architecture. This decoupling ensures that sensor drivers (e.g., BNO085, optical flow, ToF) and control logic are independent of the underlying microcontroller architecture (STM32, ESP32, RP2040, or Linux/POSIX).

```{mermaid}
flowchart TD
    subgraph Layer1 ["Layer 1: Application and Navigation Layer"]
        App1["ROS 2 Motion Planner / Nav2 Controller"]
        App2["Kinematics and Pose Estimation Engine"]
    end

    subgraph Layer2 ["Layer 2: Real-Time Middleware and Service Layer"]
        RTOS1["FreeRTOS / Zephyr Preemptive Kernel"]
        SM1["2D State Matrix Machine Framework"]
        Safe1["150 ms Hardware Deadman Watchdog Manager"]
    end

    subgraph Layer3 ["Layer 3: Platform-Independent Core Sensor Drivers - Pure C99"]
        CoreBNO["bno085_core.c: SHTP Protocol and Quaternion Math"]
        CoreEnc["encoder_core.c: 16-Bit Rollover and Metric Conversion"]
        CoreMot["motor_core.c: Linear Scaling and Saturation Math"]
    end

    subgraph Layer4 ["Layer 4: Hardware Abstraction Layer - HAL Tables"]
        HAL_I2C["hal_i2c_ops_t: I2C Bus Operations"]
        HAL_PWM["hal_pwm_ops_t: PWM Duty and Frequency"]
        HAL_TIM["hal_timer_ops_t: Timer Counter Operations"]
        HAL_GPIO["hal_gpio_ops_t: Pin IO Operations"]
    end

    subgraph Layer5 ["Layer 5: Vendor Silicon Hardware Implementations"]
        TargetSTM["STM32 HAL / LL Port: stm32f4_port.c"]
        TargetESP["ESP-IDF Port: esp32s3_port.c"]
        TargetRP["Pico SDK Port: rp2040_port.c"]
        TargetLinux["Linux devfs Port: linux_i2c_port.c"]
    end

    App1 --> RTOS1
    App2 --> CoreEnc
    App2 --> CoreBNO
    RTOS1 --> CoreMot
    CoreBNO --> HAL_I2C
    CoreMot --> HAL_PWM
    CoreMot --> HAL_GPIO
    CoreEnc --> HAL_TIM
    HAL_I2C --> TargetSTM
    HAL_PWM --> TargetSTM
    HAL_TIM --> TargetSTM
    HAL_GPIO --> TargetSTM
    HAL_I2C -.-> TargetESP
    HAL_I2C -.-> TargetRP
    HAL_I2C -.-> TargetLinux
```

#### Abstraction Interface Specification (`hal_bus_ops.h`)
The platform-independent core drivers will interact with physical hardware through virtual interface tables (function pointer structs).

```{code-block} c
/* Platform-agnostic I2C abstraction interface */
typedef struct {
    int8_t (*init)(uint32_t bus_speed_hz);
    int8_t (*read)(uint16_t dev_addr, uint8_t *data, size_t len, uint32_t timeout_ms);
    int8_t (*write)(uint16_t dev_addr, const uint8_t *data, size_t len, uint32_t timeout_ms);
    int8_t (*bus_recover)(void);
    void   (*delay_ms)(uint32_t ms);
} hal_i2c_ops_t;

/* Pure C BNO085 device handle */
typedef struct {
    const hal_i2c_ops_t *bus;
    uint16_t            address;
    bno085_reading_t    latest_data;
} bno085_dev_t;

/* Core sensor initialization using abstraction */
int8_t bno085_core_init(bno085_dev_t *dev, const hal_i2c_ops_t *bus_ops, uint16_t addr) {
    if (!dev || !bus_ops) return -1;
    dev->bus = bus_ops;
    dev->address = addr;
    return dev->bus->init(400000);
}
```

*Porting Impact:* Porting the entire robot firmware from an STM32F411 to an ESP32-S3 or Raspberry Pi RP2040 will require only implementing the 5 functions in `hal_i2c_ops_t`, leaving 100% of the SHTP parsing, 2D State Matrix logic, and dead-reckoning mathematics untouched.

### Known Limitations & Planned Enhancements
* **Open-Loop Wheel Velocity:** Current motor setpoints drive raw PWM duty cycles without closed-loop wheel velocity control on the drive motors. Future work will integrate drive motor encoders into high-rate PID/PI velocity loops in `MotorTask`.
* **Motor Driver Miniaturization (Pololu 13A Drivers):** Current builds utilize bulky Cytron MD30C drivers ($111 \times 48\text{ mm}$ each). Future chassis revisions will migrate to ultra-compact **Pololu Single-Channel 13A Motor Drivers** ($33 \times 20\text{ mm}$), yielding an ~87% footprint reduction. Bench testing on the KARMA base testbed has verified excellent thermal dissipation, high power efficiency, and seamless drop-in compatibility with the existing Sign-Magnitude PWM/DIR firmware control scheme.
* **CAN-FD Physical Transport Layer:** The USB CDC Virtual COM Port cable connection is susceptible to mechanical disconnection and electrical noise during high-acceleration arena runs. Future iterations will replace USB with an isolated CAN 2.0B / CAN-FD transceiver bus.
* **Dynamic Odometry Covariance:** Current covariance matrices published in `/odom` and `/imu/data` are fixed constants. Future firmware will calculate dynamic covariance based on tracking wheel acceleration and surface contact load.
