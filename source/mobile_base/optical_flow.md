# Optical Flow Sensor

## Overview & Objectives

The Optical Flow Sensor module provides non-contact 2D planar velocity tracking for RoboManipal's RoboCup SML 2026 mobile base. Mounted on the underside of the robot chassis facing the floor, the module combines a PixArt PMW3901 optical flow sensor with an STMicroelectronics VL53L1X Time-of-Flight (ToF) laser distance sensor.

The module operates alongside spring-loaded dead wheel odometry and a BNO085 9-DOF IMU to improve base localization. While dead wheels track mechanical displacement, optical flow measures direct ground velocity without surface contact, eliminating errors caused by wheel slip, skidding, or dead-wheel bounce during rapid robot translation.

```{figure} assets/optical_flow_assembly.jpg
:name: fig_optical_flow_assembly
:alt: Downward Facing Optical Flow and ToF Sensor Payload
:width: 80%

Downward-facing optical flow (PMW3901) and ToF (VL53L1X) sensor payload mounted on the base chassis.
```

### Core Engineering Objectives

* **Non-Contact Velocity Tracking:** Measure 2D ground-relative linear velocity $(v_x, v_y)$ independent of wheel traction or floor friction.
* **Height-Compensated Flow Scaling:** Dynamically scale PMW3901 optical pixel movement using real-time ground clearance $h$ measured by the VL53L1X ToF sensor.
* **Odometry Drift Mitigation:** Provide independent velocity feedback to correct dead wheel skidding and wheel-bounce errors during omnidirectional maneuvers.

---

## Technical Specifications & Bill of Materials (BOM)

### System Specifications

| Parameter | Specification / Value | Notes |
| --- | --- | --- |
| **Optical Flow Sensor** | PixArt PMW3901MB-TXQT | $42 \times 42$ pixel motion tracking array |
| **Distance Sensor** | STMicroelectronics VL53L1X | $940\text{ nm}$ VCSEL ToF laser sensor |
| **Primary IMU** | BNO085 9-DOF System in Package | Provides orientation and angular rates |
| **Dead Wheel Encoders** | 2x Quadrature Optical Encoders | Unpowered spring-loaded tracking wheels |
| **Operating Ground Height** | $50\text{ mm}$ to $250\text{ mm}$ | Recommended clearance range |
| **Interfaces** | SPI (PMW3901), I2C (VL53L1X) | Connected to base board microcontroller |
| **Supply Voltage** | $3.3\text{V}$ Logic / $5.0\text{V}$ Power Input | Powered from logic power rail |

### Bill of Materials (BOM)

| Designator | Component / Part Number | Manufacturer | Purpose |
| --- | --- | --- | --- |
| **U1** | PMW3901MB-TXQT | PixArt Imaging | Optical Flow Motion Tracking IC |
| **U2** | VL53L1X | STMicroelectronics | ToF Distance Ranging Sensor |
| **U3** | BNO085 | Bosch / Hillcrest Labs | 9-DOF IMU (Orientation & Gyro Rate) |
| **ENC1, ENC2** | Quadrature Encoders | Standard | Dead Wheel Displacement Tracking |

---

## Design & Architecture

### How Optical Flow is Used for Odometry

In autonomous mobile robot navigation, odometry estimates chassis displacement over time. The base uses three complementary sensors:

1. **Dead Wheels:** Track physical distance via floor-contact encoders. Highly accurate on smooth surfaces, but can slip or bounce over floor seams.
2. **BNO085 IMU:** Measures angular rate ($\omega_z$) and orientation (yaw angle), keeping robot heading aligned.
3. **PMW3901 + VL53L1X Optical Flow:** Measures visual motion across the ground plane without mechanical contact. When dead wheels skid or lose contact, optical flow provides continuous ground velocity.

### Optical Flow Velocity Calculation

The PMW3901 sensor tracks ground texture displacement between successive frame captures, outputting relative pixel counts $\Delta p_x$ and $\Delta p_y$.

Because the visual motion of ground features depends on how far the sensor is from the floor, the pixel count must be multiplied by the distance $h$ measured by the VL53L1X sensor:

$$v_x = \frac{\Delta p_x \cdot h}{f_{\text{px}} \cdot \Delta t}$$

$$v_y = \frac{\Delta p_y \cdot h}{f_{\text{px}} \cdot \Delta t}$$

Where:
* $\Delta p_x, \Delta p_y$ = Pixel displacements reported by PMW3901.
* $h$ = Ground distance measured by VL53L1X ToF sensor (in meters).
* $f_{\text{px}}$ = Calibrated optical scaling factor.
* $\Delta t$ = Time interval between sensor readings.

When combined with angular velocity ($\omega_z$) from the BNO085 IMU, robot rotation is subtracted from optical flow readings so that only true translation is used for odometry.

---

## Hardware Interface & Pinout

### Sensor Pin Connections to Base Microcontroller

| Sensor Module | Signal | MCU Connection | Description |
| --- | --- | --- | --- |
| **PMW3901** | SCLK, MOSI, MISO, CS | SPI Bus | SPI communication for motion data |
| **VL53L1X** | SCL, SDA | I2C Bus | I2C communication for distance data |
| **BNO085** | SCL, SDA / Tx, Rx | I2C / UART Bus | IMU heading and rate gyro data |
| **Dead Wheels** | Phase A, Phase B | Timer Encoder Inputs | Quadrature pulse accumulation |

---

## Future Work / Known Limitations

### Known Limitations

* **Surface Feature Dependence:** PMW3901 requires textured surfaces to track visual features; performance drops on highly reflective or featureless polished surfaces.
* **Minimum Clearance Requirement:** Ground clearance below $30\text{ mm}$ causes out-of-focus optical blur, reducing tracking accuracy.

### Next-Iteration Upgrades

* **Integrated Sensor Board:** Design a single compact breakout PCB holding both the PMW3901 and VL53L1X sensors.
* **Infrared Illumination Masking:** Optimize IR LED shroud layout to improve feature tracking on dark or low-contrast floors.
