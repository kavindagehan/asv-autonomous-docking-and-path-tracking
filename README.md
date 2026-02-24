# 🚤 Autonomous ASV Docking and Path Tracking  
### MATLAB/Simulink & ROS 2 / Gazebo Integration

This repository features a **Guidance, Navigation, and Control (GNC)** system for a **WAM-V Catamaran**.  

The project bridges the gap between **3-DOF mathematical modeling** and **high-fidelity physics simulation** in the **VRX (Virtual RobotX)** environment.

---

## ✨ Key Features

- **Adaptive lookahead radius**  
  Dynamically adjusts the tracking distance for precision maneuvering.

- **Smooth RPM profile (no spikes)**  
  Eliminates mechanical stress and jitter through optimized control shaping.

- **Distance-based docking deceleration**  
  Controlled approach speed for reliable berthing.

- **Turn-then-go speed shaping**  
  Modulates surge velocity based on heading error to ensure safety during sharp turns.

- **Real-time ROS topic integration**  
  Seamless communication between Simulink controllers and the Gazebo/VRX physics engine.

---

## 🚢 Project Highlights

### 📐 Mathematical Modeling

Implemented **3-DOF planar dynamics** for a catamaran platform, accounting for added mass and hydrodynamic drag.

Surge:
$$
m(\dot{u} - vr) = F_x - d_{11}u
$$

Sway:
$$
m(\dot{v} + ur) = F_y - d_{22}v
$$

Yaw:
$$
I_{zz}\dot{r} = N - d_{33}r
$$

---

### 🎯 Adaptive LOS Guidance

Developed a **“Rabbit” pursuit algorithm** using quadratic circle-path intersection:

\[
at^2 + bt + c = 0
\]

This enables smooth, non-oscillatory path following.

---

### ⚓ Precision Docking Logic

#### ✅ SQRT Velocity Profile
Non-linear deceleration curve:

\[
u_d \propto \sqrt{dist}
\]

Manages vessel inertia and ensures a precise *soft landing* at the dock.

#### ✅ Heading Latch
Freezes the reference heading at a **3.5 m radius** to eliminate:

- Mathematical singularities in `atan2`
- Sensor jitter during final berthing

---

### 🔁 Simulink–ROS Architecture

Real-time co-simulation using:

- MATLAB ROS Toolbox  
- GPS / IMU topic subscription  
- High-frequency thrust command publishing  

---

## 📊 Performance Results

Adaptive Lookahead guidance eliminated the **"saw-tooth" RPM spikes** common in traditional waypoint-switching logic.

| Metric | Baseline (Waypoint) | Optimized (Adaptive LOS) |
|--------|---------------------|---------------------------|
| Steering Stability | High-frequency jitter | Smooth transition |
| Docking Accuracy | Significant overshoot | Zero-overshoot stopping |
| RPM Profile | Saw-tooth oscillations | Stable cruise & ramp-down |

---

## 🛠 Tech Stack

- **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)  
- **Middleware:** ROS 2 Jazzy Jalisco  
- **Software:** MATLAB & Simulink R2025b  
- **Simulation:** Gazebo Harmonic + VRX  
- **Automation:** Python-based XML injection for 3D trajectory visualization  

---

## 🚀 Installation & Setup

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/your-username/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

### 2️⃣ MATLAB Setup

1. Open **MATLAB R2025b**
2. Install:
   - ROS Toolbox
   - Control System Toolbox
3. Add repository folders to MATLAB path:

```matlab
addpath(genpath(pwd))
```

---

### 3️⃣ ROS 2 Environment

Ensure ROS 2 Jazzy is installed:

```bash
source /opt/ros/jazzy/setup.bash
```

Launch VRX/Gazebo environment before starting Simulink co-simulation.

---

## 📂 Project Structure

```
/matlab_simulink   → Core controller models (.slx)
/scripts           → Python utilities (Gazebo XML path injection)
/docs              → Technical reports & derivations
/results           → Performance plots & simulation captures
```

---

## 👨‍🎓 Contributor

**Gehan Kavinda Dasanayake**  
Research Project – Carinthia University of Applied Sciences  

---
