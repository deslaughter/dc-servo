# dc-servo

The goal of this project is to develop a driver to control two DC gear motors from stepper motor step and direction inputs.

## Status

This project is under development and currently does the following:

- Send JSON commands via serial connection to individual servos
    - Enable logging
    - Constant Control Signal
    - Sinulsoidal Control Signal
    - Change PWM frequency
    - Set PID gains: Kp, Ki, Kd
    - Set steps to change setpoint

PID control of servo position has not been implemented.

## Hardare

- [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)
- [160W 2 Channel DC Motor Driver Module](https://www.droking.com/motor-controller/160W-2-Channel-DC-Motor-Driver-Module-Positive-Negative-PWM-Speed-Regulation-Optocoupler-Isolation-Dual-H-bridge-Motor-Controller)

## Software

- [Teensy 4.x Quad Encoder Library](https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library)
- [ArduinoJson](https://arduinojson.org/)

## Development Environment

- [Visual Studio Code](https://code.visualstudio.com/)
- [PlatformIO](https://platformio.org/)