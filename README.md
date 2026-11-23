# Differential-Drive Line Follower Robot (PID Simulator & Baseline Sprint)

A Python simulation and baseline engineering study for a 2-wheel differential-drive line follower robot equipped with a 5-channel IR sensor array.

## Overview

This repository covers the baseline sprint deliverables before hardware assembly:
1. **Concept Note**: IR array weighted-average position math, PID control intuition (why P-only oscillates and how D damps it), and a power/signal block diagram.
2. **Python PID Simulator** (`line_follower_pid_sim.py`): Simulates a 2-wheel robot with a virtual 5-IR sensor bar navigating a straight section, an S-curve, and a 90° turn.
3. **Sizing Math**: Motor RPM-to-speed calculations and proposed chassis/sensor dimensions.

## Simulation Results (P-only vs. Tuned PID)

| Controller | Gains (`Kp`, `Ki`, `Kd`) | Mean Cross-Track Error | Max Cross-Track Error | Outcome |
| :--- | :--- | :--- | :--- | :--- |
| **P-only** | `3.5`, `0.0`, `0.0` | `0.148 m` | `0.920 m` | Loops out and loses the line at the 90° turn |
| **Tuned PID** | `2.8`, `0.003`, `0.01` | `0.031 m` | `0.413 m` | Tracks the S-curve, recovers at the 90° turn, and finishes in `20.6 s` |



## Quick Start

### Prerequisites
- Python 3.9+
- `numpy` and `matplotlib`

### Install Dependencies
```bash
pip install numpy matplotlib
```

### Run the Simulator
```bash
python line_follower_pid_sim.py
```
Running the script prints the mean/max cross-track errors to the terminal and saves the 4-panel comparison plot as `line_follower_pid_comparison.png`.

## Robot & Sizing Parameters

- **Motors & Wheels**: `300 RPM` motors with `65 mm` (`0.065 m`) diameter wheels
  - Max speed: `(300 / 60) rev/s × (π × 0.065 m) ≈ 1.02 m/s`
- **Simulator Parameters**:
  - Wheelbase: `0.18 m` (`180 mm`)
  - Sensor bar forward offset: `0.12 m` (`120 mm` ahead of axle)
  - Sensor array: 5 sensors spaced across `-0.08 m` to `+0.08 m` (`40 mm` spacing in sim; `15 mm` proposed for a `20 mm` physical tape line)
  - Base speed: `0.38 m/s` (wheel speed capped at `0.55 m/s`)

## How It Works

1. **Weighted-Average Sensor Error**: Each virtual IR sensor outputs a Gaussian response based on its distance to the track centerline, combined into a normalized lateral error `e`.
2. **PID Steering Controller**: Computes angular turn rate `u = Kp·e + Ki·∫e dt + Kd·(de/dt)` using integral anti-windup and a filtered derivative term.
3. **Differential-Drive Kinematics**: Maps `u` to left/right wheel velocities (`v_L`, `v_R`), clips to `MAX_WHEEL_SPEED`, and integrates `(x, y, θ)` over `dt = 0.02 s`.
