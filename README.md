## 🛠 Requirements

### System
- **Ubuntu 24.04 LTS**
- **ROS 2 Jazzy**
- **Gazebo Harmonic**
- **VRX (Virtual RobotX) simulator** (must be installed before running co-simulation)

### MATLAB / Simulink
You will need MATLAB + Simulink and the following toolboxes (depending on your model configuration):

- Simulink
- Simulink Control Design
- Control System Toolbox
- Robotics System Toolbox (ROS 2 / ROS interface)
- Navigation Toolbox (path / guidance utilities, if used)
- Signal Processing Toolbox (filtering / smoothing, if used)
- (Add any other toolboxes you actually used in your models)

> Tip: In MATLAB, run `ver` to list installed toolboxes.

---

## 🚀 Installation & Setup

### 1️⃣ Install VRX (Required)

This project uses the **OSRF VRX simulator**. Install and verify VRX first by following the official instructions in the VRX repository:

- https://github.com/osrf/vrx

Make sure you can launch a VRX world successfully before continuing.

---

### 2️⃣ Clone This Repository

```bash
git clone https://github.com/your-username/asv-autonomous-docking-and-path-tracking.git
cd asv-autonomous-docking-and-path-tracking
```

---

### 3️⃣ ROS 2 Environment

Source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

(If you built VRX in a workspace) source your workspace as well:

```bash
source ~/vrx_ws/install/setup.bash
```

---

### 4️⃣ MATLAB Setup

1. Open MATLAB
2. Ensure required toolboxes are installed
3. Add project to path:

```matlab
addpath(genpath(pwd))
```

---

## ▶️ Run Order (Important)

1. **Start VRX / Gazebo** (ROS 2 + simulation running)
2. **Start Simulink model** (controller publishes thrust commands and subscribes to GPS/IMU)
3. Confirm ROS topics are live (GPS/IMU in, thruster commands out)

```bash
ros2 topic list
```