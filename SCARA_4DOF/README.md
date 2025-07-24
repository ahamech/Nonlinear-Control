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

1. **Put all project files in a single folder** (regardless of their original subfolder names).  
2. **Open MATLAB and set the current folder** to where you placed the files.  
3. **Run one of the following scripts**:

```matlab
scara_dynamics_sim           % For simulating a specific controller
analyze_tracking_performance % FLC vs SMC comparison and metrics
sensitivity_analysis         % Full robustness evaluation
```


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

Developed by: **Amir Hossein Akbari**  
For questions, feel free to reach out or open an issue.
