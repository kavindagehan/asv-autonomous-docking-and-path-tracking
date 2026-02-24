# Autonomous ASV Docking and Path Tracking
### MATLAB/Simulink & ROS 2 / Gazebo Integration

This repository features a Guidance, Navigation, and Control (GNC) system for a WAM-V Catamaran. The project bridges the gap between 3-DOF mathematical modeling and high-fidelity physics simulation.

## 🚢 Project Highlights
* [cite_start]**Mathematical Modeling:** Implemented 3-DOF planar dynamics accounting for added mass and hydrodynamic drag coefficients[cite: 67, 365].
* [cite_start]**Adaptive LOS Guidance:** Developed a "Rabbit" pursuit algorithm using quadratic circle-path intersection for non-oscillatory path following[cite: 172, 412].
* **Precision Docking Logic:**
    * [cite_start]**SQRT Velocity Profile:** Used a non-linear deceleration curve to manage vessel inertia and prevent overshoot[cite: 226, 251].
    * [cite_start]**Heading Latch:** Freezes reference heading at 3.5m to eliminate mathematical singularities at the docking point[cite: 243, 429].
* [cite_start]**Simulink-ROS Architecture:** Real-time co-simulation using the MATLAB ROS Toolbox to subscribe to GPS/IMU topics and publish thrust commands[cite: 565, 566].

## 🛠 Tech Stack
* [cite_start]**MATLAB & Simulink:** Controller design and path generation[cite: 13, 17].
* [cite_start]**ROS 2 & Gazebo:** High-fidelity marine environment (VRX)[cite: 550, 552].
* [cite_start]**Python:** Automated XML injection for 3D trajectory visualization in Gazebo.

## 📊 Results
[cite_start]The transition to Adaptive Lookahead guidance eliminated the "saw-tooth" RPM spikes found in traditional waypoint switching, resulting in a smooth propulsion profile and stable berthing[cite: 146, 185, 517].

## 🤝 Contributor
* [cite_start]**Gehan Kavinda Dasanayake** [cite: 11]