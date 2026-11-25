
# Self-Balancing Robot

A two-wheeled self-balancing robot developed using ESP32, MPU6050, and PID control. The robot continuously monitors its tilt angle and adjusts motor speeds in real time to maintain an upright position.

## Project Overview

The robot uses an MPU6050 accelerometer and gyroscope to measure its tilt and angular velocity. The ESP32 processes sensor data using a Kalman filter and applies PID control to calculate the required motor corrections.

The L298N motor driver controls the two BO motors to maintain the robot's balance.

## Components Used

| Component | Quantity |
|---|---|
| ESP32 | 1 |
| MPU6050 Accelerometer and Gyroscope | 1 |
| L298N Motor Driver | 1 |
| BO Gear Motors | 2 |
| LM2596 Buck Converter | 1 |
| 3.7V Li-ion Batteries | 2 |
| Robot Chassis and Wheels | 1 Set |

## Working Principle

The robot maintains its balance through continuous sensor feedback and motor correction.

1. The MPU6050 measures acceleration and angular velocity.
2. The ESP32 processes the sensor readings using a Kalman filter.
3. The PID controller calculates the correction required to maintain balance.
4. The L298N motor driver adjusts the direction and speed of both motors.
5. This process repeats continuously to keep the robot upright.

## Control System

- Sensor: MPU6050
- Sensor Fusion: Kalman Filter
- Control Algorithm: PID Controller
- Controller: ESP32
- Motor Control: L298N Dual H-Bridge Motor Driver

## Software and Programming

- Programming Language: C++
- IDE: Arduino IDE
- Microcontroller Platform: ESP32
- Sensor Communication: I2C
- Control Technique: Real-Time Feedback Control

## Features

- Two-wheeled self-balancing mechanism.
- Real-time tilt angle measurement.
- PID-based balance correction.
- Kalman filter-based sensor processing.
- Automatic motor speed and direction adjustment.
- Battery-powered operation.

## Project Demonstration

Video: https://youtu.be/Ydg5wjVDtM8

## Project Details

- Project Type: Personal Project
- Development: Independently designed, assembled, programmed, and tested.
- Semester: 2
- Domain: Robotics, Embedded Systems, and Control Systems

## Developer

Suprit Nandlaskar

GitHub: https://github.com/supritN

## License

This project is open-source and available for learning and educational purposes.
