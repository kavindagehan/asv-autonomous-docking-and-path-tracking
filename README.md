# 🚤 Autonomous ASV Docking and Path Tracking  
### MATLAB/Simulink & ROS 2 / Gazebo Integration

This repository presents a complete **Guidance, Navigation, and Control (GNC)** system for a **WAM-V Catamaran**.

The project integrates a 3-DOF vessel model with high-fidelity physics simulation in the **VRX (Virtual RobotX)** environment, enabling smooth path tracking and precision autonomous docking.

---

## ✨ Key Features

- **Adaptive Lookahead Guidance**  
  Dynamically adjusts tracking radius for smooth and stable path convergence.

- **Smooth RPM Profile (No Spikes)**  
  Eliminates oscillatory thrust behavior common in waypoint-switching methods.

- **Distance-Based Docking Deceleration**  
  Nonlinear velocity shaping ensures controlled, zero-overshoot berthing.

- **Turn-Then-Go Speed Strategy**  
  Reduces surge velocity during large heading errors for safe maneuvering.

- **Real-Time Simulink–ROS 2 Integration**  
  High-frequency thrust commands with live GPS and IMU feedback from Gazebo.

---

## 🧠 System Architecture

```
Path → Adaptive Guidance → Heading & Speed Control → Thrust Mixing → ASV Model → Feedback
```

### Control Strategy Overview

- Continuous path tracking (no discrete waypoint jumps)
- Adaptive lookahead radius near docking zone
- Heading latch during final approach to avoid instability
- Differential thrust mixing for twin-hull propulsion

---

## 📊 Performance Improvements

The Adaptive Lookahead strategy eliminates the classic **“saw-tooth” RPM oscillations** seen in baseline waypoint controllers.

| Metric | Baseline (Waypoint) | Optimized (Adaptive LOS) |
|--------|---------------------|---------------------------|
| Steering Stability | High-frequency jitter | Smooth transition |
| Docking Accuracy | Overshoot | Zero-overshoot stopping |
| RPM Behavior | Oscillatory | Stable ramp-down |

---

## 🛠 Tech Stack

- **OS:** Ubuntu 24.04 LTS  
- **Middleware:** ROS 2 Jazzy  
- **Modeling:** MATLAB & Simulink  
- **Simulation:** Gazebo Harmonic + VRX  
- **Automation:** Python (Gazebo XML path injection)

---

## 🚀 Installation & Setup

### Clone Repository

```bash
git clone https://github.com/your-username/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

### MATLAB Setup

1. Open MATLAB
2. Install:
   - ROS Toolbox  
   - Control System Toolbox  
3. Add project to path:

```matlab
addpath(genpath(pwd))
```

---

### ROS 2 Setup

Ensure ROS 2 Jazzy is installed:

```bash
source /opt/ros/jazzy/setup.bash
```

Launch the VRX simulation before running Simulink co-simulation.

---

## 📂 Project Structure

```
/matlab_simulink   → Simulink controller models
/scripts           → Gazebo XML automation tools
/docs              → Technical documentation
/results           → Plots and simulation captures
```

---

## 👨‍🎓 Author

**Gehan Kavinda Dasanayake**  
Research Project – Carinthia University of Applied Sciences

---