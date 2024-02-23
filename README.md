# mcu-solver-benchmarks

Benchmarking solvers on microcontrollers (MCU), within TinyMPC project

## Installation

* Python >= 3.9
* [OSQP v1 from source](https://github.com/osqp/osqp)
* [CVXPYGEN](https://github.com/cvxgrp/cvxpygen)
* [CVXGEN](https://cvxgen.com/docs/index.html)
* [PlatformIO](https://platformio.org/) as VSCode extension

## How to use

1. Move to `qp_mpc_problem` dir
2. Config and run `gen_mpc_problem.py`
3. Config and run `move_to_mcu_dir.py`
4. Upload and run the program with your MCU via PlatformIO

## Ready-to-run program

* Problem
  * Random QP-MPC with input constraints
* Solver
  * OSQP
  * TinyMPC
* MCU
  * STM32 (Adafruit feather board)
  * Teensy
