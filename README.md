# 🚤 Autonomous ASV Docking and Path Tracking  
### MATLAB / Simulink + ROS 2 + Gazebo (VRX)

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Model--Based%20Design-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-6E2CF2?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![VRX](https://img.shields.io/badge/VRX-Virtual%20RobotX-0B7285)](https://github.com/osrf/vrx)

A complete **Guidance, Navigation, and Control (GNC)** framework for a **WAM‑V Catamaran Autonomous Surface Vessel (ASV)**.

This project integrates:

- 3‑DOF vessel dynamics modeling  
- Adaptive Circular Line‑of‑Sight (LOS) guidance  
- PD‑based heading control  
- Surge velocity regulation  
- Differential thrust mixing (twin thrusters)  
- High‑fidelity validation in the **VRX (Virtual RobotX)** simulation environment  

✅ Smooth path tracking and precision autonomous docking **without RPM discontinuities**.

---

## 📌 Table of Contents

- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Performance Improvements](#-performance-improvements)
- [System Requirements](#-system-requirements)
- [Repository Layout](#-repository-layout)
- [Installation & Setup](#-installation--setup)
- [Execution Order](#-execution-order)
- [Dynamic Path Injection](#-dynamic-path-injection)
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
  Square‑root velocity shaping for precise, zero‑overshoot stopping.

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
  - https://github.com/osrf/vrx  

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

## 📂 Repository Layout

✅ Your modified VRX world file **must be included in this repo** (version controlled) at:

```text
vrx_world/sydney_regatta.sdf
```

Recommended structure:

```text
asv-autonomous-docking-and-path-tracking/
├── src/
│   ├── functions/
│   └── main/
├── scripts/
│   └── update_path_sydney_regatta.py
├── vrx_world/
│   └── sydney_regatta.sdf          ✅ (modified world file)
├── docs/
└── results/
```

---

## 🚀 Installation & Setup

### 1️⃣ Install VRX (Mandatory)

Follow official VRX instructions:

- https://github.com/osrf/vrx

---

### 2️⃣ Clone This Repository

```bash
git clone https://github.com/kavindagehan/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

### 3️⃣ Source ROS 2 (and VRX workspace if applicable)

```bash
source /opt/ros/jazzy/setup.bash
```

If VRX was built in a workspace:

```bash
source ~/vrx_ws/install/setup.bash
```

---

### 4️⃣ Replace VRX `sydney_regatta.sdf` (REQUIRED — do this BEFORE MATLAB Setup)

This project depends on a modified `sydney_regatta.sdf` that contains the marker tags:

```xml
<!-- START-MARKERS -->
<!-- END-MARKERS -->
```

These tags are required for **dynamic path injection**.

#### 4.1 Locate your VRX worlds directory

**Case A — VRX built from source (common):**

```bash
VRX_WORLDS_DIR=~/vrx_ws/src/vrx/vrx_gz/worlds
```

**Case B — VRX available via package share directory:**

```bash
VRX_WORLDS_DIR="$(ros2 pkg prefix vrx_gz)/share/vrx_gz/worlds"
```

Verify:

```bash
ls "$VRX_WORLDS_DIR"
```

> ⚠️ If `VRX_WORLDS_DIR` points into a system directory, you may need admin permissions to overwrite files.
> Using a workspace build (Case A) is usually the cleanest approach.

#### 4.2 Backup the original world file

```bash
cd "$VRX_WORLDS_DIR"
mv sydney_regatta.sdf sydney_regatta_original.sdf
```

#### 4.3 Copy the modified world file from this repository

Run this from the **repo root**:

```bash
cp vrx_world/sydney_regatta.sdf "$VRX_WORLDS_DIR/"
```

#### 4.4 Quick validation (confirm marker tags exist)

```bash
grep -n "START-MARKERS\|END-MARKERS" "$VRX_WORLDS_DIR/sydney_regatta.sdf"
```

✅ If both markers are printed, the world file is correctly installed.

---

### 5️⃣ MATLAB / Simulink Setup

1. Open MATLAB
2. Confirm required toolboxes are installed:

   ```matlab
   ver
   ```

3. Open your main Simulink controller model (in `/src/main` or your project’s main model directory)
4. Ensure ROS 2 network/domain settings match your VRX environment if required.

---

## ▶️ Execution Order

Use this order to avoid missing markers / missing path injection:

1. **(One-time)** Replace VRX `sydney_regatta.sdf` in the VRX environment  
2. Inject the path markers:

   ```bash
   python3 scripts/update_path_sydney_regatta.py
   ```

3. Launch VRX / Gazebo:

   ```bash
   ros2 launch vrx_gz competition.launch.py
   ```

4. Confirm ROS topics:

   ```bash
   ros2 topic list
   ```

5. Run MATLAB / Simulink controller model  
6. Verify thrust publishing:

   ```bash
   ros2 topic echo /wamv/thrusters/left/thrust
   ```

---

## 🔁 Dynamic Path Injection

Before launching the simulation:

```bash
python3 scripts/update_path_sydney_regatta.py
```

This script replaces the content between `<!-- START-MARKERS -->` and `<!-- END-MARKERS -->` in the VRX world file and injects path markers dynamically.

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