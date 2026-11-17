# Dynamic Obstacle Avoidance for Cobots using Nonlinear MPC

Real-time collision-free motion planning for a collaborative manipulator (cobot) sharing its workspace with static obstacles, multiple randomly moving obstacles, and a moving human - using **Nonlinear Model Predictive Control (NMPC)** in MATLAB. The controller drives a **KINOVA Gen3** 7-DOF arm toward its goal while continuously keeping a safe distance from every obstacle, including a human modeled as a set of collision geometries.

This project was done as part of the final-year capstone project, Dept. of Electronics & Communication Engineering, PES University, Bengaluru. Published at the **IEEE International Conference on Control, Automation and Robotics (ICCAR), 2024**:

> L. Kodandaramu, M. Srikantan, J. V. Sanoj, K. Narayan, and S. Tripathi,
> *"Dynamic Obstacle Avoidance in Cobots with Multiple Random Moving Obstacles,"* ICCAR 2024.
> [[IEEE Xplore]](https://ieeexplore.ieee.org/abstract/document/10569519)

---

## Overview

As cobots increasingly work alongside people, they need to anticipate and avoid collisions in real time. This project formulates manipulator motion planning as a **nonlinear MPC problem**: at each step the controller predicts the arm's motion over a horizon and optimizes joint accelerations to track the goal pose while enforcing a hard **minimum-safety-distance** constraint to every obstacle. The approach is tested on four cases of increasing difficulty — no obstacles, static obstacles, multiple moving obstacles, and a **human-like moving obstacle** — to show it generalizes from empty workspaces to realistic human-collaboration scenarios.

<p align="center">
  <img width="720" alt="KINOVA Gen3 avoiding a moving human (left) and multiple obstacles (right)" src="https://github.com/user-attachments/assets/f06147e7-0f75-436e-87c9-355a5d988d45" />
  <br>
  <em>The KINOVA Gen3 arm avoiding a moving human (left) and multiple static and moving obstacles (right).</em>
</p>

---

## Method

- **Manipulator & simulation.** KINOVA Gen3 (7-DOF), simulated in MATLAB with the Robotics System and Model Predictive Control toolboxes.
- **Nonlinear MPC.** The controller commands joint accelerations; the state is joint position and velocity, with simple double-integrator joint dynamics `ẋ = [0 I; 0 0]x + [0; I]u`. A quadratic cost tracks the desired end-effector pose (`Qr`), penalizes control effort (`Qu`), and adds terminal terms (`Qf`, `Qq`) for tracking and motion rate at the end of the horizon.
- **Safety as a hard constraint.** An inequality constraint (`nlmpcIneqConFunctionKINOVA.m`) runs `checkCollision` between the robot and all obstacles across the whole prediction horizon and requires every link–obstacle separation `dₙ,ₘ ≥ d_safe`; any predicted trajectory that violates it is rejected.
- **Obstacles.** Static obstacles are spheres/boxes; moving obstacles follow pre-defined trajectories (the controller only sees their current pose each step, as a sensor would).
- **Human as collision geometry.** The `atlas` humanoid model is driven through a motion via an `interactiveRigidBodyTree`; the poses of its major joints (hands, forearms, thighs, shins, head, torso) are sampled at each step and wrapped in **collision meshes** to build a person-shaped, moving obstacle the arm must avoid (`humanoid_collision_model.m`).

<p align="center">
  <img width="320" alt="Human modeled as collision meshes" src="https://github.com/user-attachments/assets/1aa04882-c7e8-4bab-a7f8-2271ae9e490b" />
  <br>
  <em>The human obstacle, built from collision meshes around the humanoid's limbs, head and torso.</em>
</p>

---

## Results

Evaluated on four cases — **Case 0:** no obstacles, **Case 1:** 3 obstacles, **Case 2:** 4 moving obstacles, **Case 3:** human-like obstacle — with position, acceleration, smoothness, torque, and cost-function analysis per joint.

- A **prediction horizon of 5** gave the best trade-off between goal-reaching speed and cost, chosen after comparing horizons of 3, 5, and 7.
- The arm reached its goal collision-free in all four cases, including around the moving human.
- **End-effector tracking error** stayed small (e.g. position error ≈ 0.01–0.03 m across cases), and **RMSE implementation error** between the NMPC commands and the executed torque-controlled motion was low (rotational ≈ 1.6–2.5°, positional ≈ 0.009–0.017), reported per case in the paper.

*(Full per-joint plots, torque/cost-function figures, and error tables are in the paper.)*

---

## Repository structure

| Folder / file | Description |
|---------------|-------------|
| `Updated_PlanAndExecute/` | Main, most up-to-date version of the pipeline. |
| `Updated_PlanAndExecute/PlanAndExecuteCollisionFreeTrajectoriesExample.mlx` | Top-level script — sets up the arm, obstacles, and NMPC, and runs the simulation. |
| `Updated_PlanAndExecute/humanoid_collision_model.m` | Builds the human as a moving collision mesh from the `atlas` humanoid model. |
| `Updated_PlanAndExecute/nlmpcIneqConFunctionKINOVA.m` | Collision-avoidance constraint (separation ≥ safety distance over the horizon). |
| `Updated_PlanAndExecute/helperCreateObstaclesKINOVA.m`, `helperUpdateMovingObstaclesKINOVA.m` | Define and animate static/moving obstacles. |
| `Updated_PlanAndExecute/nlmpc*KINOVA.m` | NMPC model, cost, and Jacobian functions for the KINOVA Gen3. |
| `DynamicObstacleAvoidance/`, `misc/` | Earlier versions and supporting scripts. |
| `PickAndPlaceWorkflowUsingStateflowForMATLABExample/` | Pick-and-place workflow the planner integrates with. |

---

## Stack

`Nonlinear MPC` · `MATLAB` · `Robotics System Toolbox` · `Model Predictive Control Toolbox` · `KINOVA Gen3` · `Collision checking`

---

## Requirements

MATLAB with the **Robotics System Toolbox** and **Model Predictive Control Toolbox** (R2021b+ recommended).

---

## Running it

Open MATLAB in `Updated_PlanAndExecute/` and run the main live script:

```matlab
open PlanAndExecuteCollisionFreeTrajectoriesExample.mlx
% toggle static vs. moving obstacles via the isMovingObst flag
```

---

## My contributions

This was a team project (5 authors). My main contributions were:
- **Implementing the nonlinear MPC with collision-avoidance constraints** for the manipulator - formulating and tuning the safety-distance inequality that keeps the arm a safe distance from every obstacle.
- **Modeling the human as a dynamic obstacle** - sampling the humanoid's limb poses over time and constructing the **collision meshes** that represent the moving person for the controller to avoid.

---

## Acknowledgement

The KINOVA Gen3 plan-and-execute and pick-and-place framework builds on MathWorks' example code, extended here with multiple moving obstacles and dynamic human-obstacle avoidance.
