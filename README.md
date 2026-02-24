# 🚤 Autonomous ASV Docking and Path Tracking  
### MATLAB / Simulink + ROS 2 + Gazebo (VRX)

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Model--Based%20Design-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-6E2CF2?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![VRX](https://img.shields.io/badge/VRX-Virtual%20RobotX-0B7285)](https://github.com/osrf/vrx)

A complete **Guidance, Navigation, and Control (GNC)** framework for a **WAM-V Catamaran Autonomous Surface Vessel (ASV)** validated in the **VRX (Virtual RobotX)** high-fidelity simulation environment.

This project integrates:

- 3-DOF vessel dynamics modeling  
- Adaptive Circular Line-of-Sight (LOS) guidance  
- PD-based heading control  
- Surge velocity regulation  
- Differential thrust mixing  
- Real-time ROS 2 communication  
- Precision autonomous docking strategy  

✅ Smooth path tracking and zero-overshoot docking without RPM discontinuities.

---

## 📌 Table of Contents

- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Performance Improvements](#-performance-improvements)
- [System Requirements](#-system-requirements)
- [Repository Layout](#-repository-layout)
- [Installation & Setup](#-installation--setup)
- [Execution Order](#-execution-order)
- [ROS Interface](#-ros-interface)
- [Notes](#-notes)
- [Author](#-author)
- [License](#-license)

---

## ✨ Key Features

- **Adaptive Circular LOS Guidance** – Continuous smooth path tracking  
- **Differential Thrust Control** – Twin-hull propulsion mixing  
- **Distance-Based Docking Deceleration** – Square-root velocity shaping  
- **Turn-Then-Go Logic** – Safe heading convergence before surge acceleration  
- **Real-Time Simulink ↔ ROS 2 Integration** – Live Gazebo sensor feedback  

---

## 🧠 System Architecture

```mermaid
flowchart LR
  P[Path / Markers] --> G[Adaptive Circular LOS]
  G --> H[PD Heading Control]
  H --> S[Surge Speed Controller]
  S --> M[Differential Thrust Mixer]
  M --> V[WAM-V Dynamics (VRX Gazebo)]
  V --> F[GPS + IMU Feedback]
  F --> G
```

---

## 📊 Performance Improvements

| Metric | Waypoint Switching | Adaptive Circular LOS |
|--------|-------------------|-----------------------|
| Steering Stability | Oscillatory | Smooth |
| Docking Accuracy | Overshoot | Zero-overshoot |
| RPM Profile | Saw-tooth | Continuous ramp |

---

# 🛠 System Requirements

## 🖥 Operating System

- Ubuntu 24.04 LTS  
- ROS 2 Jazzy  
- Gazebo Harmonic  
- VRX Simulator (vrx_gz)  
  https://github.com/osrf/vrx  

---

## 📦 MATLAB / Simulink Requirements

This project was developed and validated using:

- **MATLAB R2025b Update 3**
- **Simulink**

### 🔧 Required Toolboxes

The following toolboxes are actively used within the model and required for full functionality:

- Aerospace Blockset  
- Aerospace Toolbox  
- Automated Driving Toolbox  
- Computer Vision Toolbox  
- Control System Toolbox  
- Image Processing Toolbox  
- ROS Toolbox (ROS 2 interface)  
- Simulink Control Design  

> ⚠️ All listed toolboxes are used within the Simulink model.  
> Missing toolboxes may cause unresolved block or simulation errors.

---

### 🖥 Tested System Configuration

- Linux Kernel 6.8.x  
- ROS 2 Jazzy  
- Gazebo Harmonic  
- VRX Competition World  

---

### 🔍 Verify Installed Toolboxes

Run inside MATLAB:

```matlab
ver
```

Ensure all required toolboxes appear before running the model.

---

# 📂 Repository Layout

```text
asv-autonomous-docking-and-path-tracking/
├── src/
│   ├── functions/
│   └── main/
├── scripts/
│   └── update_path_sydney_regatta.py
├── vrx_world/
│   └── sydney_regatta.sdf   ✅ (Modified VRX world file)
├── docs/
└── results/
```

---

# 🚀 Installation & Setup

## 1️⃣ Install VRX

Follow official instructions:

https://github.com/osrf/vrx

Verify:

```bash
ros2 launch vrx_gz competition.launch.py
```

---

## 2️⃣ Clone Repository

```bash
git clone https://github.com/kavindagehan/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

## 3️⃣ Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

If built in workspace:

```bash
source ~/vrx_ws/install/setup.bash
```

---

## 4️⃣ Replace VRX `sydney_regatta.sdf` (MANDATORY – BEFORE MATLAB)

This project requires a modified `sydney_regatta.sdf` containing:

```xml
<!-- START-MARKERS -->
<!-- END-MARKERS -->
```

### Locate VRX worlds directory

Workspace build:

```bash
VRX_WORLDS_DIR=~/vrx_ws/src/vrx/vrx_gz/worlds
```

Installed package:

```bash
VRX_WORLDS_DIR="$(ros2 pkg prefix vrx_gz)/share/vrx_gz/worlds"
```

Verify:

```bash
ls "$VRX_WORLDS_DIR"
```

### Backup original

```bash
cd "$VRX_WORLDS_DIR"
mv sydney_regatta.sdf sydney_regatta_original.sdf
```

### Copy modified file from this repository

From repo root:

```bash
cp vrx_world/sydney_regatta.sdf "$VRX_WORLDS_DIR/"
```

### Confirm markers

```bash
grep -n "START-MARKERS\|END-MARKERS" "$VRX_WORLDS_DIR/sydney_regatta.sdf"
```

Both markers must appear before proceeding.

---

# ▶️ Execution Order

1. Replace VRX world file (one-time step)  
2. Inject path markers:

```bash
python3 scripts/update_path_sydney_regatta.py
```

3. Launch VRX:

```bash
ros2 launch vrx_gz competition.launch.py
```

4. Confirm ROS topics:

```bash
ros2 topic list
```

5. Open MATLAB  
6. Run Simulink controller model  
7. Verify thrust output:

```bash
ros2 topic echo /wamv/thrusters/left/thrust
```

---

# 🔁 ROS Interface

### Subscribed

- `/wamv/sensors/gps/gps/fix`
- `/wamv/sensors/imu/imu/data`

### Published

- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

---

# 📌 Notes

- Environmental disturbances (wind, waves, current) can be enabled in VRX.
- Baseline validation performed under calm conditions.
- Original world file is preserved as `sydney_regatta_original.sdf`.

---

# 👨‍🎓 Author

**Gehan Kavinda Dasanayake**  
Carinthia University of Applied Sciences  
GitHub: https://github.com/kavindagehan  

---

# 📜 License

Intended for academic and research use.  
For redistribution or commercial use, contact the author.