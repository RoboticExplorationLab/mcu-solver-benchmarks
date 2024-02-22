# QP-Based Predictive Safety Filtering

This implements a QP-based predictive safety filter for double integrator models of different dimensions. This ensures input and state trajectories within their bounds using a horizon of reference input from a nominal controller. This example is inspired from [this paper](https://arxiv.org/abs/2309.11453). It is also possible to introduce control barrier functions, control Lyapunov functions and HJ reachability.

1. Use `safety_filter.ipynb` to define problems and generate code for **TinyMPC**, **OSQP**, and **HPIPM** on computers and MCUs (Teensy and STM32). Here you can vary the dimension and horizon length of the problem. Remember to install `tinympc-python` first and checkout `TinyMPC` at `teensy_benchmark` branch.

2. Upload and run the automatically generated programs in directories such as `osqp_teensy`, `tinympc_teensy`, etc.
