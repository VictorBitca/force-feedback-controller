# FFB Controller

This controller prototype is described in more details in its dedicated [blog post](https://www.3am.engineering/works/controller-feedback/) 

In short, this project in build on top of the [FOC Arduino library](https://github.com/simplefoc/Arduino-FOC) and the [VNWheel project](https://github.com/hoantv/VNWheel), the goal was to make a custom controller dedicated for racing sims, it would provide the same force feedback functionality as racing wheel.

## Project structure

The `ffb_controller` folder contains the project files for the Arduino Leonardo microcontroller that serves as the main board, it handles all the inputs and serves as the bridge between the PC and motor controller board that bases on the Arduino Nano.

The `motor_controller` folder contains the project files for the `BGC 3.0 MOS` motor controller board, or any similar Arduino based gimbal motor driver boards.