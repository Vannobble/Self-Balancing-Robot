# 🤖 Self-Balancing Two-Wheeled Robot using MPU6050 and PID Control

![Self Balancing Robot](https://raw.githubusercontent.com/username/self-balancing-robot/main/assets/selfbalancing.jpg)

## Project Domain

This project focuses on a self-balancing robot system using the MPU6050 IMU sensor for real-time tilt detection and a PID control algorithm to maintain vertical stability. The robot uses DC motors driven by an H-bridge motor driver and is controlled via an Arduino microcontroller.

### Problem Statements

- Manual stabilization of two-wheeled robots is impossible without continuous feedback and control.
- Balancing robots are sensitive to noise, sensor drift, and motor overshoot.
- Tuning PID parameters manually without understanding system dynamics often results in instability.
- Need for low-cost, educational implementation of dynamic balancing systems.

### Goals

- Accurately detect the robot’s tilt angle in real-time using the **MPU6050** (with DMP for improved accuracy).
- Implement a **PID controller** to calculate correctional motor responses based on the tilt.
- Dynamically control two DC motors to keep the robot upright.
- Halt motor output safely when the robot falls (e.g., tilt exceeds ±45°).

### Solution Statements

- Use the **MPU6050** sensor in DMP (Digital Motion Processing) mode to obtain quaternion-based roll angle.
- Implement a PID algorithm that adjusts motor speed and direction based on the deviation from vertical (setpoint = 0° roll).
- Utilize the Arduino PWM and digital pins to control two DC motors via H-bridge (e.g., L298N).
- Add safety logic: if tilt exceeds a threshold (e.g., 45°), the motors stop and PID resets.
- Output real-time sensor data and motor control status to Serial Monitor for debugging.

---

## Feedback Control Mechanism

The control system includes:

- **Continuous Roll Angle Feedback**: Obtained from the MPU6050 using the DMP and I2Cdevlib.
- **PID Controller**:
  - **Proportional (Kp)**: Reacts to current error.
  - **Integral (Ki)**: Reacts to accumulated past errors.
  - **Derivative (Kd)**: Reacts to rate of change.
- **Output Saturation**: Limits PWM range to prevent motor overload.
- **Failsafe**: Resets PID output and stops motors if robot falls.

```cpp
// PID Parameters
double Kp = 30;
double Ki = 150;
double Kd = 1.5;
