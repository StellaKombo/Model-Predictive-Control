This repository contains the implementation of a data-driven, risk-aware algorithm designed to enable safe and efficient navigation of an autonomous drone in dynamic environments. The core of the system is a Model Predictive Control (MPC) planner enhanced with risk-aware decision-making capabilities to handle moving obstacles and environmental uncertainties in real-time.

Key components include:

Risk-Aware MPC Planner using the Gurobi optimizer.

Monte Carlo (MC) Simulations for performance evaluation under uncertainty.

MATLAB prototyping for algorithm development and validation.


Features

Real-Time Obstacle Avoidance: Achieves a 30% improvement in collision avoidance efficiency compared to baseline planners.

Risk-Aware Decision Making: Incorporates probabilistic risk models for safe navigation in dynamic, stochastic environments.

Hardware-Ready Performance: Optimized for real-time execution on an Intel i7-9700k CPU @ 3.6GHz.


Technical Stack

Languages:  MATLAB

Optimization: Gurobi Optimizer

Simulation: Monte Carlo Methods

Numerical Methods: Convex Optimization, Risk-Aware Planning, Real-Time MPC
