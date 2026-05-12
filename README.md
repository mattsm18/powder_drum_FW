# powder_drum_FW

Firmware for ENME891 - Industrial Project

# About

ENME891 is the capstone paper for the BE(Hons) degree at AUT. It is a year-long industrial project where students develop a real-world engineering solution.

### Project name: 
<b>Quanitfying the flowability of milk powders through the use of computer vision </b>

### Project members / responsibilities:
- Matthew Smith -> Embedded Firmware and Software Development, Electronic Design, on-device computer vision development
- Josh Russell -> Mechanical design and build, MATLAB Computer vision models

# Target
### Arduino Nano Every 
Potential for future, more capable targets due to using the PlatformIO framework

# Functionality
- Drive stepper motor from 0.1-70RPM with a closed-loop PID controller
- Control a 5V Relay Module
- Run bi-directional comms over serial to [powder_drum_SW](https://github.com/mattsm18/powder_drum_SW)

# Libraries

### StepperMotor
- Handles pin assignment, microstepping and internal velocity tracking
- Attaches a tick function to an ISR for atomic stepping

### AS5600 (Magnetic Encoder)
- Handles pin assignment and I2C comms with the encoder
- Determines current angular velocity of the motor
- Applies a 1st order discrete time filter

### PIController
- Wraps all PI Controller functionality
- Handles dynamic Kp, and Ki Gains + Integral Windup and deadband

# Dependencies
- Use of the PlatformIO extension in VSCode for compiling, running and debugging code

# System Diagram
![alt text](https://github.com/mattsm18/powder_drum_FW/blob/main/docs/system_diagram.png "System")

# Serial Comms Protocol
The powder drum implements a custom bi-directional serial comms protocol in 
the firmware and the software

This protocol includes ACKs, NACKs and a XOR CRC checksum to validate data integrity on the wire.

Fully implemented in the firmware, some work still to do for ACK and NACK handling in the software.

![alt text](https://github.com/mattsm18/powder_drum_FW/blob/main/docs/serial_protocol.png "Custom Serial Protocol")