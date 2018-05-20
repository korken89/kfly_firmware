# The code and specifications for KFly Dronecode

This is mostly a complete rewrite/redesign of the old KFly code into C++17 and utilizing and testing the [CRECT scheduler](https://github.com/korken89/crect) in real-world usage.

Will also be the testing ground for a forward looking COBS ZP/ZRE generator.

### Hardware Specifications

All PCBs follow the standard 30.5 x 30.5 mm mounting holes, but the external form-factor is changeable.

#### KFly Dronecode PCB:
* ST's **STM32F765** CPU: 32-bit ARM Cortex-M7F @ 216 MHz with 64-bit FPU, DSP and advanced branch prediction
* Bosch's **BMI160** accelerometer / gyroscope
* ST's **LPS22HB** pressure sensor
* Dronecode compatible
    * Separate analog and digital power supply with 5.3 V input voltage specification (dronecode)
    * Most inputs and outputs are dronecode compatible
* 8 outputs (50 Hz / 400 Hz / Oneshot125 / Oneshot42 / Multishot / Dshot)
    * Supports ESC telemetry
    * All outputs on easy to use 3-pin JST-GH
* MicroSD card holder for "Blackbox" flight logging

