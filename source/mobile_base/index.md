# Mobile Base

The mobile base platform provides omnidirectional mobility, localization, power distribution, and real-time drive control for RoboManipal's RoboCup Smart Manufacturing League (SML) robot. 

The platform utilizes a 4-wheel Mecanum drive in an X-configuration, supported by spring-loaded dead-wheel odometry pods, an optical flow ground velocity sensor, a high-current power distribution PCB, and embedded motor controllers running FreeRTOS.

```{toctree}
:maxdepth: 2
:caption: Platform Modules

drivetrain
odometry_pods
power_distribution
motor_control
optical_flow
sensor_fusion
navigation/index
```
