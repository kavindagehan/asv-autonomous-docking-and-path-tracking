📌 Overview

This repository presents the modeling, guidance, and control implementation of a 3-DOF Autonomous Surface Vehicle (ASV) based on a catamaran platform.

The system integrates:

MATLAB/Simulink dynamic modeling

Adaptive Lookahead LOS guidance

PD-based heading and speed control

Thrust mixing for differential propulsion

ROS–Gazebo (VRX) high-fidelity validation

The controller was validated in the Virtual RobotX (VRX) simulation environment.

🧠 System Architecture

Path → Guidance → Heading & Speed Control → Thrust Mixing → ASV Model → Feedback

Key features:

Adaptive lookahead radius

Smooth RPM profile (no spikes)

Distance-based docking deceleration

Turn-then-go speed shaping

Real-time ROS topic integration