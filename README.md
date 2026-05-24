# Fuel Consumption Optimization of a Parallel Mild-Hybrid Vehicle using MATLAB/Simulink

Energy Management System (EMS) for a **parallel mild-hybrid electric vehicle (P2 HEV)**, comparing a **Rule-Based Strategy (RBS)** against an optimal **Dynamic Programming (DP)** approach. The models are built in **MATLAB/Simulink** and evaluated over the **NEDC** and **FTP-75** driving cycles, using a Mercedes-Benz A 170 CDI as the reference platform.

> Originally developed as a seminar project for *Electromobility (SS 2023)* at the Institute of Electromobility, RPTU Kaiserslautern-Landau.

---

## Table of Contents

- [Overview](#overview)
- [Problem Definition](#problem-definition)
- [Vehicle Model & Specifications](#vehicle-model--specifications)
- [Methodology](#methodology)
  - [Rule-Based Strategy (RBS)](#rule-based-strategy-rbs)
  - [Dynamic Programming (DP)](#dynamic-programming-dp)
- [Results](#results)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Conclusion](#conclusion)
- [Future Scope](#future-scope)
- [References](#references)
- [Authors](#authors)
- [License](#license)

---

## Overview

In a hybrid electric vehicle, an Energy Management System decides how propulsion torque is split between the internal combustion engine and the electric motor at every instant. A good split lowers fuel consumption while keeping the battery charge-sustaining over the cycle.

This project implements and benchmarks two EMS approaches on the same vehicle model:

1. **Rule-Based Strategy (RBS)** — a real-time, heuristics-driven controller using load-point shifting and electric-driving rules.
2. **Dynamic Programming (DP)** — a globally optimal, offline benchmark that minimizes a fuel-cost function over the full cycle subject to charge-sustaining constraints.

Both are simulated over two standard cycles (NEDC, FTP-75) and compared against a conventional (non-hybrid) baseline.

---

## Problem Definition

- **Goal:** Minimize equivalent fuel consumption of a parallel mild-hybrid vehicle while sustaining the battery State of Charge (SOC) across a driving cycle.
- **Control variable:** Torque split ratio between the combustion engine and the electric motor.
- **Constraints:** SOC bounded within operating limits; charge-sustaining (start SOC ≈ end SOC ≈ 50%).
- **Evaluation cycles:** NEDC and FTP-75.

---

## Vehicle Model & Specifications

Reference platform: **Mercedes-Benz A 170 CDI (W168), ~1115 kg**

| Component | Specification |
|---|---|
| Combustion engine | Diesel OM 622 — 60 kW, 187 Nm @ 4200 rpm, 1698 cm³ |
| Electric motor | Permanent Magnet Synchronous Motor (PMSM) — 12 kW, 60 Nm, 7639 rpm |
| Battery | Lithium-ion — 16.38 kW, 0.468 kWh, 46.8 V, 13 mΩ |
| Gearbox | 5-speed manual |
| Clutch | Friction clutch between combustion engine and electric motor |
| Architecture | Parallel mild-hybrid (P2) |

---

## Methodology

The overall workflow models the vehicle once, then runs both EMS strategies on it across both driving cycles for a like-for-like comparison.

### Rule-Based Strategy (RBS)

A real-time controller whose rules are derived from heuristics, engineering intuition, and the vehicle's operating maps. Key mechanisms:

- **Load Point Shifting (LPS)** — shifts the engine operating point toward higher-efficiency regions.
- **Electric Driving (ED)** — uses the electric motor alone in suitable low-load conditions.
- **Equivalent Fuel Consumption (EFC)** accounting to keep electrical and fuel energy comparable.
- Control logic governs the **torque split ratio** and the **engine on/off state** based on SOC and load demand.

### Dynamic Programming (DP)

A globally optimal benchmark that discretizes the state space and computes the minimum-cost torque-split trajectory over the full cycle, subject to initial and final boundary conditions. Implemented in MATLAB/Simulink using the generic DPM function by Sundström & Guzzella (2009).

Representative DPM grid configuration:

| Parameter | NEDC | FTP-75 |
|---|---|---|
| SOC discretization (`grd.Nx`) | 101 | 101 |
| SOC bounds (`Xn.lo` / `Xn.hi`) | 0.15 / 0.95 | 0.15 / 0.95 |
| Split-ratio discretization (`grd.Nu`) | 20001 | 10001 |
| Split-ratio bounds (`Un.lo` / `Un.hi`) | -1 / 1 | -1 / 1 |
| Initial SOC (`grd.X0`) | 0.50 | 0.50 |
| Final SOC range (`XN.lo` / `XN.hi`) | 0.501 / 0.51 | 0.501 / 0.51 |

> DP requires full prior knowledge of the cycle, so it serves as an optimal offline reference rather than a real-time controller.

---

## Results

Fuel consumption is reported in litres/100 km against the conventional-vehicle baseline. Both strategies sustained SOC (~18000 coulombs, i.e. ~50%) at cycle end.

**Rule-Based Strategy vs. Conventional**

| Driving Cycle | Conventional (L/100 km) | RBS (L/100 km) | Fuel Reduction |
|---|---|---|---|
| NEDC | 4.897 | 3.559 | **27.32 %** |
| FTP-75 | 4.675 | 3.272 | **30.01 %** |
| Average | 4.786 | 3.415 | **28.66 %** |

**Dynamic Programming vs. Conventional**

| Driving Cycle | Conventional (L/100 km) | DP (L/100 km) | Fuel Reduction |
|---|---|---|---|
| NEDC | 4.897 | 3.270 | **33.22 %** |
| FTP-75 | 4.675 | 2.998 | **35.87 %** |
| Average | 4.786 | 3.134 | **34.54 %** |

**Key takeaway:** Dynamic Programming delivered roughly **6 % higher** average fuel-consumption reduction than the Rule-Based Strategy, while both maintained charge-sustaining operation.

> 🏆 This project was awarded **1st place** at the RPTU Kaiserslautern Energy Management seminar.

---

## Repository Structure

> Update the paths/filenames below to match your actual repository layout.

```
.
├── README.md
├── RuleBasedStrategy/        # Simulink models & scripts for the RBS controller
├── DynamicProgramming/       # DPM function setup, cost-function & DP controller
├── VehicleModel/             # Plant model: engine, PMSM, battery, drivetrain
├── DriveCycles/              # NEDC and FTP-75 cycle data
├── Results/                  # Plots: fuel consumption, SOC, torque split
└── docs/                     # Seminar presentation / report
```

---

## Getting Started

### Prerequisites

- MATLAB (R2021a or later recommended)
- Simulink
- Simscape / Powertrain-related toolboxes (as required by the plant model)

### Running the Simulation

1. Clone the repository:
   ```bash
   git clone https://github.com/Rahul-Chid/Fuel_Consumption_Optimization_using_MATLAB_Simulink.git
   ```
2. Open the project folder in MATLAB and add it to the path.
3. Load a driving cycle (NEDC or FTP-75) from `DriveCycles/`.
4. Run the **Rule-Based Strategy** model to reproduce the RBS results.
5. Run the **Dynamic Programming** setup to reproduce the optimal benchmark.
6. Compare outputs (fuel consumption, SOC, torque split) in `Results/`.

---

## Conclusion

This project provides an in-depth comparative study of a Rule-Based Strategy and a Dynamic Programming approach as energy management strategies for a parallel mild-hybrid vehicle over the NEDC and FTP-75 cycles. The Rule-Based Strategy achieved an average fuel-consumption reduction of **28.66 %**, while Dynamic Programming reached **34.54 %** — about 6 % higher. Both strategies maintained ~50 % SOC at the start and end of each cycle, confirming effective, charge-sustaining battery management.

---

## Future Scope

- Incorporate gear-ratio selection into the rule-based logic to refine efficiency.
- Replace DP's reliance on full future cycle knowledge with adaptive / machine-learning models that predict and adapt to real-time driving conditions.
- Extend the study to a broader range of vehicle parameters and alternative advanced EMS approaches.

---

## References

1. O. Sundström and L. Guzzella, "A generic dynamic programming MATLAB function," *2009 IEEE Control Applications (CCA) & Intelligent Control (ISIC)*, pp. 1625–1630. doi: 10.1109/CCA.2009.5281131
2. N. Xu et al., "Towards a Smarter Energy Management System for Hybrid Vehicles: A Comprehensive Review of Control Strategies," *Applied Sciences*, 2019, pp. 1–38.
3. T. Hofmann, M. Steinbuch, R. M. van Druten, A. F. A. Serrarens, "Rule-Based Energy Management Strategies for Hybrid Electric Drivetrains," *IFAC Proceedings Volumes*, 2006, vol. 39, pp. 740–745.
4. Y. Tian, J. Liu, Q. Yao, K. Liu, "Optimal Control Strategy for Parallel Plug-in Hybrid Electric Vehicles Based on Dynamic Programming," *World Electric Vehicle Journal*, 2021, pp. 0–17.
5. C. C. Chan and Y. S. Wong, "The state of the art of electric vehicles technology," *IPEMC 2004*, vol. 1, pp. 46–57.
6. O. Govardhan, "Fundamentals and Classification of Hybrid Electric Vehicles," *Int. Journal of Engineering and Techniques*, 2017, vol. 3, pp. 194–198.

---

## Authors

Seminar Electromobility SS 2023 — Group 8, M.Sc. Commercial Vehicle Technology, RPTU Kaiserslautern-Landau:

- **Rahul Chidambaranathan** — [GitHub](https://github.com/Rahul-Chid) · [LinkedIn](https://www.linkedin.com/in/rahul-chidambaranathan-0777a41aa/)
- Danish Shahid Pathan
- Srinivas Shanmuga Sundaram

---

## License

Released under the MIT License. See [`LICENSE`](LICENSE) for details.

> *Add a `LICENSE` file to the repository if one is not already present.*
