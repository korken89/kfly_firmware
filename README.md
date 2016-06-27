#The code and specifications for the KFly PCB version 3 and above

### Hardware Specifications

All PCBs follow the standard 36 x 36 mm outline with 30.5 x 30.5 mm mounting holes, unless nothing else is stated.

#### KFly PCB version 3:
* ST's **STM32F405** CPU: 32-bit ARM Cortex-M4F @ 168 MHz with 32-bit FPU and DSP
* Invensense's **MPU-6050** accelerometer / gyroscope
* Honeywell's **HMC5983** magnetometer
* Meas. Spec.'s **MS5611** pressure sensor
* 8 outputs (50 Hz / 400 Hz / Oneshot125)
* 4 expansion connectors (3 UARTs & 1 CAN port)
* 6 slot RC input connector with support for (C)PPM up to 8 channels in and signal strength
* 433 MHz RF link for data, command and control transfers

#### KFly PCB version 4:
* ST's **STM32F746** CPU: 32-bit ARM Cortex-M7F @ 216 MHz with 32-bit FPU, DSP and advanced branch prediction
* ST's **LSM6DS3** accelerometer / gyroscope (changed due to its low gain error)
* ST's **LIS3MDL** magnetometer
* ST's **LPS25HB** pressure sensor
* Separate analog and digital power supply with 6 V max input voltage
* 8 outputs (50 Hz / 400 Hz / Oneshot125)
* 4 expansion connectors (3 UARTs & 1 CAN port)
* 6 slot RC input connector with support for (C)PPM and SBUS with up to 18 channels and signal strength indication
* MicroSD card holder for "Blackbox" flight logging


### Software Specifications
#### Common features
* Runs ChibiOS 16.x
* USB Serial Bootloader - no need for an external programmer!
* Quaternion based Square-Root Multiplicative Extended Kalman Filter (SR-MEKF) provides estimation
* "Control signal to Motor"-mixing for different types of frames
* Flight control: Rate mode, Attitude mode and more to come

#### Cortex-M7 features
* The Cortex-M7 based (v. 4+) will be running Model Predictive Controllers (MPCs)
