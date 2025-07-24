# SCARA 4-DOF Robot – Nonlinear Control Project

This project implements nonlinear control for a 4-DOF SCARA robotic manipulator based on the dynamic model presented in [this paper](https://doi.org/10.1017/S0263574723000450).

## ✅ Features
- Symbolic dynamic modeling using Lagrange formulation
- Feedback Linearization Control (FLC)
- Sliding Mode Control (SMC)
- Trajectory tracking and disturbance rejection
- Sensitivity and robustness analysis

## 📁 Folder Structure
- `dynamics/`: Computes M(q), C(q,dq), G(q), and Phi
- `controllers/`: FLC and SMC implementations
- `simulation/`: Main simulation scripts
- `plots/`: Output figures (position, error, torque, etc.)
- `results/`: RMSE, control energy, max errors
- `report/`: Technical write-up of the project

## 🛠 Requirements
- MATLAB R2022+ (Symbolic Math Toolbox)
- Tested on Windows/Linux

## 👨‍💻 Author
Amir Hossein Akbari
