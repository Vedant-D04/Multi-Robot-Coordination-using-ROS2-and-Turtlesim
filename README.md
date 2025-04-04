# Gesture-Controlled Multi-Robot Coordination for Automated Room Cleaning (ROS2, TurtleSim)

This project implements a **gesture-controlled multi-robot coordination system** for **automated room cleaning** using **ROS2 Jazzy** on **Ubuntu 24.04 (VM)**. The system utilizes **MediaPipe for hand gesture recognition** and **ROS2 for robot coordination** in a simulated cleaning environment using **TurtleSim**.

## 🚀 Features
- **Gesture-based control** using OpenCV & MediaPipe  
- **Multi-robot coordination** to optimize cleaning efficiency  
- **Collision avoidance & task distribution** for smooth robot movement  
- **ROS2-based real-time communication** for robot control  

---

## 🛠 Installation

### 1. Install ROS2 Jazzy
Follow the official ROS2 installation guide:  
[ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)

### 2. Install Required Dependencies
```bash
sudo apt update && sudo apt install -y \
    python3-pip ros-jazzy-turtlesim \
    ros-jazzy-rclpy ros-jazzy-rclcpp \
    ros-jazzy-std-msgs ros-jazzy-nav2-bringup

pip install opencv-python mediapipe
```
## ▶ Running
```bash
# Navigate to your ROS2 workspace
cd ~/coordination

# Build the project
colcon build

# Source the environment
source install/setup.bash

# Launch the multi-robot coordination system
ros2 launch multi_robot_coordination multi_robot_system.launch.py
```
## 🎯 System Overview

The system consists of three key modules:

### 🖐 Gesture Control System
- Uses **MediaPipe hand tracking** and **OpenCV** to detect user gestures.
- Recognizes:
  - **Open Palm** → Starts cleaning process.
  - **Closed Fist** → Stops all robot movement.
- Sends real-time commands to cleaning robots via **ROS2 publisher-subscriber messaging**.

### 🤖 Multi-Robot Coordination Module
- Dynamically assigns **cleaning tasks** to multiple robots.
- Prevents overlapping movement paths using **ROS2-based inter-robot communication**.
- Implements **collision avoidance algorithms** for smooth navigation.

### 🏠 Cleaning Execution and Environment Simulation
- **Five robots** are randomly spawned in the environment.
  - **Three robots** move randomly to clean the floor.
  - **Two stationary robots** represent **dirt spots**, disappearing when cleaned.

### 📸 Visual Representation
Below are images showcasing the **TurtleSim environment before and after cleaning**, demonstrating the effectiveness of the system.

![Before Cleaning]
![PHOTO-2025-03-28-10-50-04 2](https://github.com/user-attachments/assets/5a613eba-c544-42e1-b224-37619ebfbe45)

*The environment before the cleaning process starts.*

![After Cleaning]
![PHOTO-2025-03-28-10-50-03 3](https://github.com/user-attachments/assets/dc1bf995-73a7-4761-9e69-587f9a71b7d0)

*The environment after the robots have cleaned the dirt spots.*

