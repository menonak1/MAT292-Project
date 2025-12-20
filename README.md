# Quadcopter Attitude Simulation

Python simulation of a quadcopter using a 12-state Newton–Euler model with PD/PID control. The main script `Code simulator.py` runs a hover and step-attitude test, logging state histories and plotting altitude/attitude and motor thrusts.

## Requirements
- Python 3.9+
- Packages: `numpy`, `scipy`, `matplotlib`

Install once:
```bash
pip install numpy scipy matplotlib
```

## To run the simulation
From the project root:
```bash
python "Code simulator.py"
```
Results in 2 plots being generated
- Position subplot (z only) with desired altitude reference
- Attitude subplot (roll, pitch, yaw) and motor thrusts

## What the sim does
- Integrates the 12-DOF rigid-body ODEs with `scipy.integrate.solve_ivp` (RK45)
- Inner PD loops for roll, pitch, yaw; PID for altitude
- Control allocation via an invertible mixing matrix to compute individual motor thrusts
- Step commands for roll/yaw (and optional pitch) between 2–8 s; hover at 1 m

## Tuning tips
- Gains live in the "Initialize Controllers" section of `Code simulator.py`
- Start with Kp only, add Kd for damping, then small Ki (altitude only)
- Adjust `dt` and `total_time` to trade fidelity vs. run time

## LaTeX report
The write-up lives in `ODE Final Project.tex` with references in `references.bib`.

## Folder layout
- `Code simulator.py` — main simulation script
- `class PIDController.py` — PID class
- `Latex docs/` — report and bibliography
