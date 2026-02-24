# 🚤 Autonomous ASV Docking and Path Tracking  
### MATLAB / Simulink + ROS 2 + Gazebo (VRX)

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-6E2CF2?logo=gazebo&logoColor=white)](https://gazebosim.org/)

A complete **Guidance, Navigation, and Control (GNC)** framework for a **WAM‑V Catamaran Autonomous Surface Vessel (ASV)**, validated in the **VRX (Virtual RobotX)** simulation environment.

This project integrates:

- 3‑DOF vessel dynamics modeling  
- Adaptive Circular Line‑of‑Sight (LOS) guidance  
- PD‑based heading control  
- Surge velocity regulation  
- Differential thrust mixing  
- High‑fidelity simulation in **Gazebo (VRX)** with **ROS 2** feedback  

✅ Achieves smooth path tracking and precision autonomous docking **without RPM discontinuities**.

---

## 📌 Table of Contents

- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Performance Improvements](#-performance-improvements)
- [System Requirements](#-system-requirements)
- [Installation & Setup](#-installation--setup)
- [VRX World File Configuration](#-vrx-world-file-configuration)
- [Dynamic Path Injection](#-dynamic-path-injection)
- [Execution Order](#-execution-order)
- [Project Structure](#-project-structure)
- [ROS Interface](#-ros-interface)
- [Notes](#-notes)
- [Author](#-author)
- [License](#-license)

---

## ✨ Key Features

- **Adaptive Circular LOS Guidance**  
  Continuous path progression without waypoint switching spikes.

- **Smooth Differential Thrust Profile**  
  Eliminates saw‑tooth RPM oscillations caused by discrete waypoint jumps.

- **Distance‑Based Docking Deceleration**  
  Square‑root velocity shaping for precise, zero‑overshoot stopping near the dock.

- **Turn‑Then‑Go Strategy**  
  Reduces surge speed during large heading errors for safe maneuvering.

- **Real‑Time Simulink ↔ ROS 2 Integration**  
  Live GPS and IMU feedback from Gazebo with real‑time thrust publishing.

---

## 🧠 System Architecture

### High‑Level Flow

```mermaid
flowchart LR
  P[Path / Markers] --> G[Adaptive Circular LOS Guidance]
  G --> H[PD Heading Control]
  H --> S[Surge Speed Control]
  S --> M[Differential Thrust Mixing]
  M --> V[WAM-V Dynamics in Gazebo (VRX)]
  V --> F[Sensor Feedback: GPS + IMU]
  F --> G
```

### Control Design Highlights

- Continuous spline‑based path tracking  
- Adaptive lookahead radius near docking zone  
- Heading latch for final approach stability  
- Differential thrust control for twin‑hull propulsion  

---

## 📊 Performance Improvements

The Adaptive LOS strategy removes classical RPM spikes seen in waypoint‑switching controllers.

| Metric | Baseline (Waypoint Switching) | Adaptive Circular LOS |
|--------|-------------------------------|------------------------|
| Steering Stability | High‑frequency jitter | Smooth heading transition |
| Docking Accuracy | Overshoot | Zero‑overshoot stop |
| RPM Behavior | Saw‑tooth oscillations | Smooth ramp‑down |

---

## 🛠 System Requirements

### 🖥 Operating System

- Ubuntu 24.04 LTS  
- ROS 2 Jazzy  
- Gazebo Harmonic  
- VRX Simulator (`vrx_gz`)  
  - Repository: https://github.com/osrf/vrx  

---

### 📦 MATLAB / Simulink Requirements

- MATLAB (R2025b or compatible)  
- Simulink  
- **ROS Toolbox** (ROS 2 communication)  
- Control System Toolbox  
- Simulink Control Design  

Optional (if used):

- Navigation Toolbox  
- Signal Processing Toolbox  

Check installed toolboxes:

```matlab
ver
```

---

## 🚀 Installation & Setup

### 1) Install VRX (Mandatory)

Follow the official VRX instructions:

- https://github.com/osrf/vrx

Verify Gazebo launches:

```bash
ros2 launch vrx_gz competition.launch.py
```

---

### 2) Clone This Repository

```bash
git clone https://github.com/kavindagehan/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

### 3) Source ROS 2 Environment

```bash
source /opt/ros/jazzy/setup.bash
```

If VRX was built in a workspace:

```bash
source ~/vrx_ws/install/setup.bash
```

---

## 🗺 VRX World File Configuration

This project requires a modified base version of `sydney_regatta.sdf`.

The world file **must** contain the following marker tags:

```xml
<!-- START-MARKERS -->
<!-- END-MARKERS -->
```

These tags enable **dynamic path injection**.

### One‑Time Setup

Navigate to the VRX world directory:

```bash
cd ~/vrx_ws/src/vrx/vrx_gz/worlds/
```

Backup the original file:

```bash
mv sydney_regatta.sdf sydney_regatta_original.sdf
```

Copy the modified base world file from this repository:

```bash
cp vrx_world/sydney_regatta.sdf ~/vrx_ws/src/vrx/vrx_gz/worlds/
```

---

## 🔁 Dynamic Path Injection

Before launching the simulation, run:

```bash
python3 scripts/update_path_sydney_regatta.py
```

This script replaces the content between `START-MARKERS` and `END-MARKERS` and injects path markers dynamically.

---

## ▶️ Execution Order

1. Launch VRX / Gazebo:

   ```bash
   ros2 launch vrx_gz competition.launch.py
   ```

2. Confirm ROS topics:

   ```bash
   ros2 topic list
   ```

3. Open MATLAB / Simulink.

4. Run the main controller model (Simulink).

5. Verify thrust publishing:

   ```bash
   ros2 topic echo /wamv/thrusters/left/thrust
   ```

---

## 📂 Project Structure

```text
/src
 ├── functions
 ├── main
 └── scripts

/vrx_world
/docs
/results
```

---

## 🔁 ROS Interface

### Subscribed Topics

- `/wamv/sensors/gps/gps/fix`
- `/wamv/sensors/imu/imu/data`

### Published Topics

- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

---

## 📌 Notes

- Environmental disturbances (wind, waves, current) can be enabled in VRX for robustness testing.
- Validation was performed under still‑water conditions for baseline benchmarking.
- The original VRX world file is preserved as a backup (`sydney_regatta_original.sdf`).

---

## 👨‍🎓 Author

**Gehan Kavinda Dasanayake**  
GitHub: https://github.com/kavindagehan  
Research Project – Carinthia University of Applied Sciences

---

## 📜 License

This repository is intended for **academic and research purposes**.  
If you plan to reuse or redistribute this work beyond academic use, please contact the author.