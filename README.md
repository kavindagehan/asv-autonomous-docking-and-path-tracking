Autonomous ASV Docking and Path Tracking

MATLAB/Simulink & ROS 2 / Gazebo Integration

This repository features a Guidance, Navigation, and Control (GNC) system for a WAM-V Catamaran. The project bridges the gap between 3-DOF mathematical modeling and high-fidelity physics simulation.

🚢 Project Highlights

Mathematical Modeling: Implemented 3-DOF planar dynamics for a catamaran platform, accounting for added mass and hydrodynamic drag coefficients.

Adaptive LOS Guidance: Developed a "Rabbit" pursuit algorithm using quadratic circle-path intersection for smooth, non-oscillatory path following.

Precision Docking Logic:

SQRT Velocity Profile: Utilized a non-linear deceleration curve to manage vessel inertia and ensure a precise "soft landing" at the dock.

Heading Latch: Freezes the reference heading at a 3.5m radius to eliminate mathematical singularities and sensor jitter during the final berthing approach.

Simulink-ROS Architecture: Real-time co-simulation using the MATLAB ROS Toolbox to subscribe to navigation topics (GPS/IMU) and publish high-frequency thrust commands.

🛠 Tech Stack

Operating System: Ubuntu 24.04 LTS (Noble Numbat)

Middleware: ROS 2 Jazzy Jalisco

Software: MATLAB & Simulink R2025b

Simulation: Gazebo Harmonic / VRX (Virtual RobotX)

Automation: Python-based XML injection for 3D trajectory visualization in Gazebo.

📊 Results

The implementation of Adaptive Lookahead guidance successfully eliminated the "saw-tooth" RPM spikes commonly found in traditional waypoint-switching logic. This resulted in a significantly smoother propulsion profile, reduced mechanical stress on thrusters, and stable vessel behavior during complex docking maneuvers.

🤝 Contributor

Gehan Kavinda Dasanayake

Research Project - Carinthia University of Applied Sciences