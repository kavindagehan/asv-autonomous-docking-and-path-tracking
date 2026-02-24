# 🚤 Autonomous ASV Docking and Path Tracking  
### MATLAB / Simulink + ROS 2 + Gazebo (VRX)

This repository presents a complete **Guidance, Navigation, and Control (GNC)** system for a **WAM-V Catamaran**.

The project integrates a 3-DOF vessel model with high-fidelity physics simulation in the **VRX (Virtual RobotX)** environment, enabling smooth path tracking and precision autonomous docking.

---

# ✨ Key Features

- **Adaptive Lookahead Guidance**  
  Continuous path tracking without waypoint switching spikes.

- **Smooth RPM Profile (No Saw-Tooth Oscillations)**  
  Eliminates mechanical stress caused by discontinuous thrust commands.

- **Distance-Based Docking Deceleration**  
  Nonlinear velocity shaping for zero-overshoot berthing.

- **Turn-Then-Go Speed Strategy**  
  Reduces surge velocity during large heading errors for safe maneuvering.

- **Real-Time Simulink–ROS 2 Integration**  
  Live GPS and IMU feedback from Gazebo with high-frequency thrust publishing.

---

# 🧠 System Architecture

```
Path → Adaptive Guidance → Heading Controller (PD) → Speed Controller → Thrust Mixing → WAM-V Model → Feedback
```

### Control Overview

- Continuous spline-based path tracking  
- Adaptive lookahead radius near docking zone  
- Heading latch during final approach  
- Differential thrust mixing for twin-hull propulsion  

---

# 📊 Performance Improvements

The Adaptive Lookahead strategy removes the classic **RPM spikes** found in traditional waypoint-switching controllers.

| Metric | Baseline (Waypoint) | Optimized (Adaptive LOS) |
|--------|---------------------|---------------------------|
| Steering Stability | High-frequency jitter | Smooth transition |
| Docking Accuracy | Overshoot | Zero-overshoot stop |
| RPM Behavior | Saw-tooth oscillations | Stable ramp-down |

---

# 🛠 Requirements

## 🖥 System Requirements

- **Ubuntu 24.04 LTS**
- **ROS 2 Jazzy**
- **Gazebo Harmonic**
- **VRX Simulator (Required)**  
  https://github.com/osrf/vrx

---

## 📦 MATLAB / Simulink Requirements

- MATLAB (R2025b or compatible)
- Simulink
- Robotics System Toolbox (ROS 2 interface)
- Control System Toolbox
- Simulink Control Design
- Navigation Toolbox (if used for path utilities)
- Signal Processing Toolbox (if used for filtering)

You can check installed toolboxes with:

```matlab
ver
```

---

# 🚀 Installation & Setup

---

## 1️⃣ Install VRX (Mandatory First Step)

This project depends on the OSRF VRX simulator.

Follow the official installation instructions:

👉 https://github.com/osrf/vrx

Verify VRX runs correctly before continuing:

```bash
ros2 launch vrx_gz competition.launch.py
```

(Ensure Gazebo launches successfully.)

---

## 2️⃣ Clone This Repository

```bash
git clone https://github.com/your-username/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

## 3️⃣ Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

If VRX was built in a workspace:

```bash
source ~/vrx_ws/install/setup.bash
```

---

## 4️⃣ MATLAB Setup

1. Open MATLAB
2. Ensure required toolboxes are installed
3. Add project to path:

```matlab
addpath(genpath(pwd))
```

---

# ▶️ Execution Order (Important)

1. **Launch VRX / Gazebo**
2. Confirm ROS topics are active:

```bash
ros2 topic list
```

3. **Open Simulink model**
4. Start simulation (controller begins publishing thrust commands)
5. Verify thruster topics:

```bash
ros2 topic echo /wamv/thrusters/left/thrust
```

---

# 📂 Project Structure

```
/matlab_simulink   → Simulink controller models (.slx)
/scripts           → Python Gazebo XML automation tools
/docs              → Technical documentation
/results           → Plots and simulation captures
```

---

# 🔁 ROS Interface

### Subscribed Topics
- `/wamv/sensors/gps/gps/fix`
- `/wamv/sensors/imu/imu/data`

### Published Topics
- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

---

# 👨‍🎓 Author

**Gehan Kavinda Dasanayake**  
Research Project – Carinthia University of Applied Sciences  

---

# 📌 Notes

- Environmental disturbances (wind, waves, currents) can be enabled in VRX if robustness testing is required.
- The controller was validated in still-water conditions for baseline performance benchmarking.

---