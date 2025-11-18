This repository contains the software and hardware documentation for a Mecanum Wheel Mobile Robot Platform. The platform utilizes a TB6612FNG Dual DC Motor Driver, DC motors with encoder feedback, and a suite of sensors for obstacle avoidance and control. The current configuration is based on an Arduino microcontroller.

# Key Features:

# Omnidirectional Mobility:
Implemented using four Mecanum wheels for 360-degree movement.

# Motor Control & Feedback:
Utilizes motor encoders for closed-loop speed control and stall detection.

# Reactive Contact:
Includes logic to detect and respond to motor or wheel stalls/stuck events.

# Obstacle Avoidance:
Integrated Ultrasonic Sensor for distance measurement and collision prevention.

# Actuator:
Features a Servo Motor for potential use with an attached mount, sensor mount, or steering mechanism.
 
# Hardware & Components
Components------------------------------ Quantity

DC motor with encoders----------------------4

TB6612FNG Motor Driver----------------------2

Mecanum Wheels------------------------------4

Arduino Microcontroller---------------------1

Ultrasonic Sensor (HC-SR04)-----------------1

Servo Motor---------------------------------1

# Circuit Diagram & Connections

The primary connections involve the TB6612FNG drivers, DC motors, and Arduino. Note: Due to limitations in the circuit modeling software (Fritzing), the Encoder, Servo Motor, and Ultrasonic Sensor connections are not explicitly displayed in the provided diagram but are referenced in the code.

# Encoder Connections

The motor encoders require connection to the Arduino's Interrupt Pins for accurate speed and position tracking. The code expects two channels (A and B) per motor (8 total channels) connected to a combination of digital and interrupt-capable pins.

Note: The specific pin assignments for the encoders are left to the developer and must be accurately reflected in the Arduino code (.ino file) to match the final hardware configuration.
