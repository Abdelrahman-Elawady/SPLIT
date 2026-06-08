<div align="center">
  
# 🤖 Project S.P.L.I.T.
**Snapping Pendulums Literally In Two**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)]([INSERT_LICENSE_LINK_OR_REMOVE])
[![C++](https://img.shields.io/badge/Language-C++-00599C.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Language-Python-FFD43B.svg)](https://www.python.org/)
[![PlatformIO](https://img.shields.io/badge/Built_with-PlatformIO-orange.svg)](https://platformio.org/)

*A highly advanced, self-balancing robotic swarm architecture utilizing distributed Dual-MCU processing, cascaded PID control, and computer vision for high-density indoor micro-logistics.*

[INSERT_IMAGE_OR_GIF_LINK_HERE - e.g., A wide shot of the robots balancing or the Web UI]

**[🎥 Watch the Full Project Video on YouTube]([https://youtu.be/ys7Rda4Vqj8?si=qLUs55d14E2go74t])**

</div>

---

## 📖 Table of Contents
1. [About the Project](#-about-the-project)
2. [Distributed Hardware Architecture](#-distributed-hardware-architecture-dual-mcu)
3. [Control Theory & Physics](#-control-theory--physics)
4. [Computer Vision & Teleoperation](#-computer-vision--teleoperation)
5. [Repository Structure](#-repository-structure)
6. [Hardware BOM](#-hardware-bom)
7. [Getting Started (Installation)](#-getting-started-installation)
8. [Future Improvements](#-future-improvements)
9. [Team & Acknowledgments](#-team--acknowledgments)

---

## 🚀 About the Project

Modern warehouse facilities face a critical spatial bottleneck: traditional four-wheeled logistics robots require massive floor space and wide aisles to maneuver safely. 

**Project S.P.L.I.T.** is a self-balancing robotic swarm developed at the **University of Prince Mugrin (UPM)**. By navigating exclusively on two wheels as inverted pendulums, these robots drastically reduce their physical footprint. This allows industrial facilities to narrow their aisles and maximize high-density storage without sacrificing payload capacity.

### Key Features:
* **Dynamic Stability:** A custom 200 Hz cascaded PID control system executing real-time sensor fusion via a mathematically optimized Kalman filter.
* **Distributed MCU Processing:** Decoupling high-speed physics calculations from network routing using two distinct ESP32 microcontrollers per robot.
* **Real-time Web Dashboard:** A WebSockets-based telemetry dashboard hosted on a local SPIFFS server for live PID tuning and auto-scaling data visualization.
* **Computer Vision Navigation:** Off-board OpenCV processing using ArUco markers for autonomous pathfinding and MediaPipe for hand-gesture teleoperation.

---

## 🧠 Distributed Hardware Architecture (Dual-MCU)

One of the most fatal flaws in DIY robotics is asking a single microcontroller to handle both high-speed physics calculations and heavy wireless network routing. Pausing for just a few milliseconds to process a Wi-Fi packet causes a balancing robot to violently crash.

We solved this by establishing a **Distributed Dual-MCU Architecture** governed by a central Bridge router:

1. **The Drive MCU (Physics Engine):** An ESP32 dedicated entirely to survival. Its Wi-Fi radio is physically disabled. It reads the MPU6050 IMU, tracks the quadrature encoders, and executes the 200 Hz balancing math.
2. **The Cortex MCU (The Brain):** A secondary ESP32 that handles payload actuation (Servo Gripper) and acts as the communications relay. It communicates with the Drive MCU via a 500,000 baud hardware UART connection, utilizing a custom `0xAA 0xAA` header packet to prevent data corruption.
3. **The Bridge MCU (The Router):** A central hub plugged into the control station. It hosts a local Wi-Fi Access Point and Web Server, routing traffic to the robot swarm via the ultra-low-latency ESP-NOW protocol.

---

## ⚖️ Control Theory & Physics

Keeping a two-wheeled robot upright requires absolute mathematical precision. To maintain equilibrium, we implemented a **Cascaded PID (Proportional-Integral-Derivative) Controller**.

The system relies on three simultaneous control loops working in harmony:
* **Inner Angle Loop (200 Hz):** Calculates the difference between the robot's current tilt and the target angle, outputting raw PWM signals to the motor drivers to prevent falling.
* **Outer Speed/Position Loop (10 Hz):** Reads high-resolution wheel encoders. If the robot drifts, it calculates a position error and shifts the *Target Angle* of the inner loop, causing the robot to lean and drive to the correct location.
* **Yaw Loop (200 Hz):** Reads the Z-axis gyroscope to ensure motors spin in perfect synchronization, preventing unintentional spinning during rapid accelerations.

> **Note on Sensor Fusion:** We bypassed the MPU6050's built-in DMP to avoid processing delays. Instead, we pull raw accelerometer and gyroscope data via a 400kHz I2C bus and run it through our own **Kalman Filter** to neutralize motor vibrations and correct gyroscope drift.

---

## 👁️ Computer Vision & Teleoperation

Because onboard cameras would shift the center of mass, we offloaded vision processing to an external global system running on a laptop (The "Eye in the Sky").

* **ArUco Pathfinding:** A Python script utilizing OpenCV detects distinct ArUco markers on each robot. It calculates the robot's exact X/Y coordinates and Yaw (heading direction), generating forward throttle and steering vectors to route the robot to its destination.
* **Hand Gesture Control:** Integrated **MediaPipe Hand Tracking** maps 21 3D landmarks of a human hand. The system translates physical hand gestures into direct kinematic commands, allowing operators to direct the swarm seamlessly.

---

## 📁 Repository Structure

```text
SPLIT-Robotics-Swarm/
├── firmware/                      # Embedded C++ code for the ESP32s
│   ├── bridge_controller/         # Main bridge router code & Web UI
│   │   ├── bridge_controller.ino  
│   │   └── data/                  # SPIFFS Web Files (HTML/CSS/JS)
│   ├── cortex_controller/         # ESP-NOW relay and Servo actuator
│   └── drive_controller/          # 200Hz PID and Kalman filter math
│
├── software/                      # Off-board applications (Python)
│   └── computer_vision/           
│       ├── main_tracker.py        # ArUco tracking and vector math
│       ├── gesture_control.py     # MediaPipe hand gesture script
│       └── requirements.txt       # Python dependencies
│
├── hardware/                      # Physical design files
│   ├── 3D_models/                 # STL files for chassis and gripper
│   └── electronics/               # Wiring diagrams and schematics
│
├── testing/                       # Isolated scripts for calibration
│   ├── motor_test/             
│   └── imu_calibration/           # Script to find MPU6050 offsets
│
└── docs/                          # Images, posters, and documentation