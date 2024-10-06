# Sim-to-Real Waffle Bot Implementation
This repository contains the code for implementing a waffle bot's learned wall-following behavior in a real-world environment. The project focuses on transitioning from a Gazebo simulation (used in a previous phase) to a physical robot platform.

[![Simulation Vs Real World Demo](https://img.youtube.com/vi/kvWROpc9Lk4/0.jpg)](https://www.youtube.com/watch?v=kvWROpc9Lk4)

Key Features:
- Sim-to-Real Transfer: Adapted the simulated robot's learned behavior to real-world conditions, eliminating Gazebo-specific dependencies.
- Q-Table Integration: Utilized a pre-trained Q-table from reinforcement learning to guide the robot's decision-making without additional training.
- Laser Scanner Optimization: Adjusted laser scanner data handling to account for real-world anomalies (e.g., converting 0 distances to infinity).

The real-world implementation exhibited similar performance to the simulation, with slight adjustments needed for physical constraints like proximity to walls and turns.
