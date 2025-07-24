# SCARA 4-DOF Robotic Manipulator 🚀

This folder contains a complete MATLAB implementation of modeling, control, and analysis for a **4-DOF SCARA robotic manipulator**. The system is nonlinear and includes multiple control strategies for trajectory tracking and robustness analysis.

---

## 📦 Structure

```
SCARA_4DOF/
├── Dynamics/           % Dynamic modeling (M, C, G, Φ)
├── Reference/          % Desired trajectory generation (sine / step)
├── Controllers/        % FLC and SMC control laws and dynamics
├── Simulation/         % Simulation scripts for running the robot
├── Analysis/           % Performance and robustness analysis
```

---

## 🎯 Features

- ✅ Full dynamic model of a 4-DOF SCARA robot using Lagrangian formulation
- ✅ Modular implementation: Modeling, simulation, and control are separated
- ✅ Implements two nonlinear controllers:
  - **Feedback Linearization Control (FLC)**
  - **Sliding Mode Control (SMC)**
- ✅ Handles **external disturbances** and **trajectory tracking**
- ✅ Full **sensitivity analysis** over parameters, initial states, and disturbances
- ✅ RMSE, Max Error, and Control Energy evaluation

---

## 🛠️ How to Run

Make sure to add all subfolders to MATLAB path before running simulations. This is already handled at the top of each main script:

```matlab
addpath(genpath(fileparts(mfilename('fullpath'))));
```

Then, you can run simulations via:

```matlab
scara_dynamics_sim           % For individual controller
scara_dynamics_sim_compare   % FLC vs SMC comparison
analyze_tracking_performance % RMSE, MaxError, Energy plots
sensitivity_analysis         % Full robustness evaluation
```

---

## 🧪 Dependencies

- MATLAB R2021a or later recommended
- No toolboxes required

---

## 📎 Citation

This implementation is inspired by the modeling and control setup from:

**"Nonlinear optimal control for a 4-DOF SCARA robotic manipulator"**  
*Robotica*, 2023. [DOI: 10.1017/S0263574723000450]

> This repository provides an **independent** MATLAB implementation of the SCARA system and controllers, designed for **educational and academic** use.

---

## 📬 Contact

Developed by: **Amirhossein Akbari**  
For questions, feel free to reach out or open an issue.
