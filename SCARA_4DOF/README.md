# SCARA 4-DOF Robot – Nonlinear Control Project

This folder contains the complete MATLAB implementation of a nonlinear control system for a 4-DOF SCARA robotic manipulator.

## ✅ Highlights
- Dynamic modeling based on Lagrangian formulation
- Controllers:
  - Feedback Linearization Control (FLC)
  - Sliding Mode Control (SMC)
- Full simulation and performance analysis
- Sensitivity analysis under parametric, initial, and disturbance variations

## 🧩 Folder Structure
- `dynamics/`: Model setup and equations of motion
- `reference/`: Desired trajectory generator
- `controllers/`: FLC and SMC implementations
- `simulation/`: Simulation scripts and controller switch
- `analysis/`: RMSE, control energy, robustness testing
- `plots/`, `results/`, `report/`: Optional outputs

## 📎 Citation
Based on: [Nonlinear optimal control for a 4-DOF SCARA robotic manipulator](https://doi.org/10.1017/S0263574723000450)

## 🛠 Requirements
- MATLAB R2022 or later
- Symbolic Math Toolbox
