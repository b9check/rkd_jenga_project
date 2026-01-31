# Jenga Tower Robot

Autonomous 18-block Jenga tower stacking using a 5-DOF HEBI robot arm. Final project for Robot Kinematics & Dynamics (CMU).

## Overview

Programmed a robot arm to autonomously pick blocks from a fixed location and stack a complete 18-block Jenga tower (6 layers, alternating orientations). The system runs at 100Hz, commanding both position and velocity to the motors.

## What I Implemented

**Kinematics**
- Forward kinematics using DH parameters and homogeneous transforms
- Closed-form inverse kinematics via kinematic decoupling — derived geometric solution separating position (joints 1-3) from orientation (joints 4-5)

**Trajectory Generation**
- Trapezoidal velocity profiles: acceleration ramp → constant velocity cruise → deceleration ramp. Computed max velocity from time/distance constraints, integrated to get position.
- Cubic spline interpolation for smooth multi-waypoint motions (zero velocity at endpoints)

**Controls**
- Tuned per-joint PID gains to minimize overshoot and settling time for precise block placement

## File Structure

```
├── Robot.m                 # Robot class: FK, IK, Jacobians
├── pick_place_sample.m     # Main execution: sequencing, waypoints, control loop
├── trajectory_trap_vel.m   # Trapezoidal velocity trajectory generation
├── trajectory_spline.m     # Cubic spline trajectory generation
├── DH_ALL.m                # DH parameter definitions
├── forward_kin.m           # Forward kinematics helper
├── jenga_gains.mat         # Tuned PID gains
└── lib/hebi/               # HEBI robotics SDK
```

## Hardware

- HEBI 5-DOF robot arm (base, shoulder, elbow, wrist1, wrist2)
- Suction gripper
- 100Hz control loop over serial

## Technical Details

**Inverse Kinematics Approach**

Used kinematic decoupling to solve IK analytically:
1. Compute wrist center by subtracting gripper offset from goal position
2. Solve joints 1-3 geometrically (2-link planar arm in the radial-vertical plane)
3. Solve joints 4-5 from orientation constraint

**Trapezoidal Trajectory**

Given start/end positions and total time T:
- Duty cycle determines fraction of time spent accelerating/decelerating
- Max velocity: `v_max = distance / (T - ramp_time)`
- Ramp phases: linearly increase/decrease velocity, integrate for position
- Cruise phase: constant velocity interpolation
