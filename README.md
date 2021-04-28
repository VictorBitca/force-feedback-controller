# FFB Controller

This controller prototype is described in more details in its dedicated [blog post](https://www.3am.engineering/works/controller-feedback/) 

In short, this project is build on top of the [FOC Arduino library](https://github.com/simplefoc/Arduino-FOC) and the [VNWheel project](https://github.com/hoantv/VNWheel), the goal was to make a custom controller dedicated to racing sims, it would provide the same force feedback functionality as a racing wheel.

## Project structure

The `ffb_controller` folder contains the project files for the Arduino Leonardo microcontroller that serves as the mainboard, it handles all the inputs and serves as the bridge between the PC and Arduino Nano based motor controller board.

The `motor_controller` folder contains the project files for the `BGC 3.0 MOS` motor controller board or any similar Arduino gimbal motor driver boards.
