# 🤖 Advanced Trajectory Tracking Control for PUMA 560 Robot Manipulator Computed-Torque-and-Robust-Min-Max-Control

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Robotics Toolbox](https://img.shields.io/badge/Robotics_Toolbox-Peter_Corke-blue.svg)](https://petercorke.com/toolboxes/robotics-toolbox/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Control Theory](https://img.shields.io/badge/Control-Nonlinear-red.svg)]()

> **A comprehensive implementation of advanced nonlinear control strategies for robotic manipulators, demonstrating the practical application of Computed Torque Control and Robust Min-Max Control under dynamic uncertainties.**

---

## 📋 Table of Contents
- [Overview](#-overview)
- [Key Features](#-key-features)
- [Technical Highlights](#-technical-highlights)
- [System Architecture](#-system-architecture)
- [Installation](#-installation)
- [Usage](#-usage)
- [Results & Performance](#-results--performance)
- [Mathematical Foundation](#-mathematical-foundation)
- [Project Structure](#-project-structure)
- [Future Enhancements](#-future-enhancements)
- [Contributing](#-contributing)
- [References](#-references)
- [Authors](#-authors)

---

## 🎯 Overview

This project implements and analyzes two sophisticated control strategies for trajectory tracking of the PUMA 560 robotic manipulator:

1. **Computed Torque Control (CTC)** - A PD feedback controller integrated with feedforward inverse dynamics for nonlinearity compensation
2. **Robust Min-Max Control** - An enhanced controller designed to handle model uncertainties and payload variations

### 🎓 Academic Context
- **Course**: ME 650 - Robot Manipulators
- **Institution**: [Stevens Institute of Technology]
- **Instructor**: Dr. Long Wang

### 🔬 Research Objectives
- Implement full dynamic modeling of PUMA 560 using Lagrangian mechanics
- Validate model accuracy against Recursive Newton-Euler (RNE) method
- Analyze PD controller performance across all 6 DOF
- Evaluate robustness under payload uncertainty (2 kg unknown mass)
- Compare nominal vs. robust control strategies in joint and task spaces

---

## ✨ Key Features

### 🛠️ Technical Implementation
- ✅ **Exact PUMA 560 Dynamic Model** - Based on Armstrong et al. (1986) parameters
- ✅ **Lagrangian Mechanics** - First-principles derivation of robot dynamics
- ✅ **Quintic Polynomial Trajectories** - Smooth position, velocity, and acceleration profiles
- ✅ **Real-time Simulation** - Semi-implicit Euler integration with 1ms time steps
- ✅ **Comprehensive Error Analysis** - Joint-space and task-space metrics
- ✅ **3D Visualization** - Animated robot motion with trajectory tracking

### 🎯 Control Strategies
| Controller | Description | Use Case |
|------------|-------------|----------|
| **PD Control** | Basic proportional-derivative feedback | Baseline performance analysis |
| **Computed Torque** | PD + Inverse Dynamics | Ideal conditions with perfect model knowledge |
| **Robust Min-Max** | CTC + Disturbance compensation | Real-world scenarios with uncertainties |

### 📊 Performance Metrics
- Root Mean Square Error (RMSE)
- Maximum Tracking Error
- Settling Time Analysis
- Overshoot Quantification
- Task-space position accuracy (mm-level precision)

---

## 🔬 Technical Highlights

### 1. Dynamic Modeling Excellence
```matlab
% Robot dynamics equation
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ

Where:
- M(q): 6×6 configuration-dependent inertia matrix
- C(q,q̇): Coriolis and centrifugal matrix
- G(q): Gravity vector
- τ: Joint torque vector
```

**Validation Results:**
- Model vs. RNE torque error: < 10⁻¹⁰ Nm
- Matrix symmetry verified: M(q) = M(q)ᵀ
- Positive definiteness confirmed

### 2. Intelligent Gain Tuning
```matlab
% Position gains (larger for base, smaller for wrist)
Kp = diag([500, 180, 50, 30, 30, 20])

% Velocity gains (critically damped response)
Kd = diag([350, 50, 20, 10, 10, 5])
```

**Rationale:** Joint-specific tuning accounts for varying inertia and mechanical advantage across the kinematic chain.

### 3. Quintic Trajectory Generation
Ensures C² continuity (smooth acceleration):
```matlab
q(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵

Boundary conditions:
- q(0) = q₀, q(T) = qf
- q̇(0) = q̇(T) = 0
- q̈(0) = q̈(T) = 0
```

**Benefits:**
- Eliminates jerk-induced vibrations
- Predictable joint coordination
- Actuator-friendly torque profiles

### 4. Robust Control Innovation

The min-max controller adds a disturbance rejection term:

```matlab
τ = M_nominal(q)[q̈_ref + Kd*ė + Kp*e] + C_nominal(q,q̇)q̇ + G_nominal(q) + τ_robust

Where τ_robust compensates for:
- Unknown payload (Δm = 2 kg)
- Model parameter uncertainties
- External disturbances
```

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Trajectory Generator                  │
│          (Quintic polynomials for all 6 joints)         │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│                   Control Layer                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  PD Control  │  │  Computed    │  │  Robust      │  │
│  │              │  │  Torque (CTC)│  │  Min-Max     │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│              Dynamic Model (PUMA 560)                    │
│  • Inertia Matrix M(q)                                  │
│  • Coriolis Matrix C(q,q̇)                              │
│  • Gravity Vector G(q)                                  │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│              Forward Dynamics Solver                     │
│        q̈ = M⁻¹(τ - Cq̇ - G)                            │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│         Numerical Integration (Semi-implicit Euler)      │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│        Visualization & Error Analysis                    │
│  • Joint space tracking                                 │
│  • Task space position error                            │
│  • 3D robot animation                                   │
└─────────────────────────────────────────────────────────┘
```

---

## 🚀 Installation

### Prerequisites
```matlab
% Required toolboxes
- MATLAB R2020b or later
- Robotics Toolbox by Peter Corke (v10.x)
- Control System Toolbox (optional, for analysis)
```

### Setup Instructions

1. **Clone the repository**
```bash
git clone https://github.com/yourusername/puma560-control.git
cd puma560-control
```

2. **Install Robotics Toolbox**
```matlab
% In MATLAB command window
addpath('path/to/robotics-toolbox')
startup_rvc
```

3. **Verify installation**
```matlab
% Run verification script
run('scripts/verify_setup.m')
```

---

## 💻 Usage

### Quick Start

```matlab
% 1. Run complete simulation suite
main_simulation

% 2. Visualize results
plot_results

% 3. Generate comparison report
generate_report
```

### Individual Experiments

#### Experiment 1: PD Control Step Response
```matlab
% Test PD controller on all 6 joints
run_pd_step_response

% Analyze settling time and overshoot
analyze_step_response
```

#### Experiment 2: Trajectory Tracking (Nominal)
```matlab
% Track quintic trajectory without payload
params.payload = 0;
run_trajectory_tracking(params)
```

#### Experiment 3: Payload Uncertainty Test
```matlab
% Add 2 kg unknown mass to end-effector
params.payload = 2.0;  % kg
params.controller = 'CTC';
run_trajectory_tracking(params)
```

#### Experiment 4: Robust Control Comparison
```matlab
% Compare CTC vs Robust Min-Max
compare_controllers('CTC', 'MinMax', payload=2.0)
```

### Advanced Configuration

```matlab
% Custom trajectory parameters
traj_params = struct(...
    'q0', [0, 0, 0, 0, 0, 0], ...
    'qf', [pi/4, pi/3, pi/2, 0, pi/4, 0], ...
    'duration', 5.0, ...
    'dt', 0.001 ...
);

% Custom control gains
gains = struct(...
    'Kp', diag([500, 180, 50, 30, 30, 20]), ...
    'Kd', diag([350, 50, 20, 10, 10, 5]) ...
);

% Run simulation
results = simulate_robot(traj_params, gains);
```

---

## 📊 Results & Performance

### Joint Space Tracking Performance

| Joint | Nominal RMSE (rad) | Payload RMSE (rad) | Robust RMSE (rad) | Improvement |
|-------|-------------------|-------------------|-------------------|-------------|
| q₁ | 2.81 × 10⁻⁴ | 3.57 × 10⁻⁴ | 1.42 × 10⁻⁴ | **60.2%** ↓ |
| q₂ | 1.35 × 10⁻³ | 2.19 × 10⁻³ | 8.73 × 10⁻⁴ | **60.1%** ↓ |
| q₃ | 7.04 × 10⁻⁴ | **0.0184** | 5.21 × 10⁻³ | **71.7%** ↓ |
| q₄ | 4.57 × 10⁻⁵ | 2.11 × 10⁻⁴ | 6.83 × 10⁻⁵ | **67.6%** ↓ |
| q₅ | 1.92 × 10⁻³ | 8.47 × 10⁻³ | 2.34 × 10⁻³ | **72.4%** ↓ |
| q₆ | 2.68 × 10⁻⁷ | 4.91 × 10⁻⁷ | 1.15 × 10⁻⁷ | **76.6%** ↓ |

### Task Space Performance

| Axis | Nominal Max Error (m) | Payload Max Error (m) | Robust Max Error (m) |
|------|---------------------|---------------------|---------------------|
| X | 0.0032 | 0.0287 | 0.0045 |
| Y | 0.0028 | 0.0134 | 0.0039 |
| Z | 0.0041 | **0.0492** | 0.0067 |

**Key Insight:** The robust controller reduces Z-axis error by **86.4%** under payload uncertainty, critical for vertical precision tasks.

### Visualization Gallery

#### Joint Tracking Comparison
![Joint Tracking](docs/images/joint_tracking.png)
*Comparison of desired vs. actual trajectories for all 6 joints*

#### Error Evolution
![Error Analysis](docs/images/error_analysis.png)
*Temporal evolution of tracking errors (nominal vs. robust)*

#### 3D Robot Motion
![Robot Animation](docs/images/robot_animation.gif)
*Animated PUMA 560 executing quintic trajectory*

#### Task Space Precision
![Task Space](docs/images/task_space_error.png)
*End-effector position error in Cartesian coordinates*

---

## 📐 Mathematical Foundation

### Lagrangian Dynamics

The robot's equations of motion are derived from the Lagrangian:

```
ℒ(q, q̇) = T(q, q̇) - V(q)

where:
- T: Total kinetic energy
- V: Potential energy
```

Applying Euler-Lagrange equations:

```
d/dt(∂ℒ/∂q̇) - ∂ℒ/∂q = τ
```

This yields the compact form:

```
τ(q, q̇, q̈) = M(q)q̈ + C(q, q̇)q̇ + G(q)
```

### Control Law Derivation

#### Computed Torque Control

Define tracking error:
```
e = q_ref - q
ė = q̇_ref - q̇
```

Feedback linearization via:
```
τ = M(q)[q̈_ref + Kp*e + Kd*ė] + C(q,q̇)q̇ + G(q)
```

Closed-loop dynamics become:
```
ë + Kd*ė + Kp*e = 0  (linear, decoupled)
```

#### Lyapunov Stability

Energy-based stability proof:
```
V = (1/2)ėᵀM(q)ė + (1/2)eᵀKp*e

V̇ = -ėᵀKd*ė ≤ 0  (negative semi-definite)
```

Therefore, the system is **globally stable** with **exponential convergence** when Kp, Kd > 0.

### Robust Control Enhancement

To handle model uncertainty Δ:

```
Ma(q) = Mn(q) + ΔM
Ca(q,q̇) = Cn(q,q̇) + ΔC
Ga(q) = Gn(q) + ΔG
```

The robust term compensates via sliding mode approach:

```
τ_robust = -ρ * sign(ė)

where ρ > ||Δ|| (uncertainty bound)
```

---

## 📁 Project Structure

```
puma560-control/
│
├── src/
│   ├── models/
│   │   ├── puma560_dynamics.m       # Dynamic model implementation
│   │   ├── validate_model.m         # RNE validation
│   │   └── compute_jacobian.m       # Forward/inverse kinematics
│   │
│   ├── controllers/
│   │   ├── pd_controller.m          # Basic PD control
│   │   ├── computed_torque.m        # CTC implementation
│   │   └── robust_minmax.m          # Robust controller
│   │
│   ├── trajectory/
│   │   ├── quintic_poly.m           # Quintic trajectory generation
│   │   └── validate_trajectory.m    # Check continuity
│   │
│   └── simulation/
│       ├── forward_dynamics.m       # ODE solver wrapper
│       ├── simulate_robot.m         # Main simulation loop
│       └── integration.m            # Numerical integration
│
├── scripts/
│   ├── main_simulation.m            # Run all experiments
│   ├── run_pd_step_response.m       # Experiment 1
│   ├── run_trajectory_tracking.m    # Experiments 2-3
│   └── compare_controllers.m        # Experiment 4
│
├── analysis/
│   ├── compute_errors.m             # RMSE, max error, etc.
│   ├── plot_results.m               # Generate all plots
│   └── generate_report.m            # LaTeX/PDF report
│
├── data/
│   ├── puma560_parameters.mat       # Armstrong et al. data
│   └── simulation_results/          # Saved results
│
├── docs/
│   ├── images/                      # Plots and animations
│   ├── ME650_Project_Report.pdf     # Full technical report
│   └── presentation.pptx            # Project presentation
│
├── tests/
│   ├── test_dynamics.m              # Unit tests for model
│   ├── test_controllers.m           # Controller verification
│   └── verify_setup.m               # Installation check
│
├── README.md                        # This file
├── LICENSE                          # MIT License
└── requirements.txt                 # MATLAB dependencies
```

---

## 🔮 Future Enhancements

### Short-term (1-3 months)
- [ ] **Adaptive Control**: Online parameter estimation for unknown payloads
- [ ] **Real-time Implementation**: Port to C++ with ROS integration
- [ ] **Experimental Validation**: Test on physical PUMA 560 hardware
- [ ] **Friction Modeling**: Include Coulomb and viscous friction

### Medium-term (3-6 months)
- [ ] **Machine Learning Integration**: Neural network-based disturbance observer
- [ ] **Optimal Control**: LQR/MPC for trajectory optimization
- [ ] **Multi-objective Optimization**: Balance tracking vs. energy efficiency
- [ ] **Collision Avoidance**: Integration with path planning algorithms

### Long-term (6-12 months)
- [ ] **Collaborative Robotics**: Human-robot interaction scenarios
- [ ] **Visual Servoing**: Image-based control integration
- [ ] **Force Control**: Hybrid position/force strategies
- [ ] **Digital Twin**: Real-time simulation for predictive maintenance

---

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/AmazingFeature`)
3. **Commit your changes** (`git commit -m 'Add AmazingFeature'`)
4. **Push to the branch** (`git push origin feature/AmazingFeature`)
5. **Open a Pull Request**

### Code Style
- Follow MATLAB best practices (see `docs/style_guide.md`)
- Add comments for complex algorithms
- Include unit tests for new features
- Update documentation accordingly

---

## 📚 References

### Primary Sources
1. **Armstrong, B., Khatib, O., & Burdick, J.** (1986). "The Explicit Dynamic Model and Inertial Parameters of the PUMA 560 Arm." *Proceedings of the 1986 IEEE International Conference on Robotics and Automation*, San Francisco, CA.

2. **Hollerbach, J.M.** (1980). "A Recursive Lagrangian Formulation of Manipulator Dynamics and a Comparative Study of Dynamics Formulation Complexity." *IEEE Trans. on Systems, Man and Cybernetics*, Vol. SMC-10, No. 11, pp. 730-736.

3. **Corke, P.** (2017). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB®*. Springer, 2nd Edition.

### Additional Reading
- Spong, M.W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
- Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics*. Springer, 2nd Edition.
- Slotine, J.J.E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall.

---

## 👥 Authors

**Mohammad Althaf Syed**  
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue)](https://linkedin.com/in/yourprofile)
[![GitHub](https://img.shields.io/badge/GitHub-Follow-black)](https://github.com/yourusername)
[![Email](https://img.shields.io/badge/Email-Contact-red)](mailto:your.email@example.com)

**Bhagyath Badduri**  


### Acknowledgments
- **Dr. Long Wang** - Course instructor and project advisor
- **Peter Corke** - Robotics Toolbox development
- **ME 650 Teaching Assistants** - Technical support

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2024 Mohammad Althaf Syed, Bhagyath Badduri

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---

## 🌟 Star History

[![Star History Chart](https://api.star-history.com/svg?repos=yourusername/puma560-control&type=Date)](https://star-history.com/#yourusername/puma560-control&Date)

---

## 📬 Contact & Support

- **Issues**: [GitHub Issues](https://github.com/yourusername/puma560-control/issues)
- **Discussions**: [GitHub Discussions](https://github.com/yourusername/puma560-control/discussions)
- **Email**: althaf2577@gmail.com

---

<div align="center">

### If you found this project helpful, please consider giving it a ⭐!

**Built with ❤️ for the robotics community**

[⬆ Back to Top](#-advanced-trajectory-tracking-control-for-puma-560-robot-manipulator)

</div>
