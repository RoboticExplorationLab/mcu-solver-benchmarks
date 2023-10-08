# mcu-solver-benchmarks

Within TinyMPC project

## Installation

* Python 3.9
* [OSQP v1 from source](https://github.com/osqp/osqp)
* [CVXPYGEN](https://github.com/cvxgrp/cvxpygen)
* [CVXGEN](https://cvxgen.com/docs/index.html)
* [PlatformIO](https://platformio.org/) as VSCode extension

## How to use

1. Config your MCU with PlatformIO
2. Move to `rand_prob_gen` dir
3. Config and run `gen_mpc_problem.py`
4. Config and run `move_to_mcu_dir.py`
5. Upload and run the program with your MCU

## Ready-to-run program

* Problem
  * Random QP-MPC with input constraints
* Solver
  * OSQP
  * TinyMPC
* MCU
  * Teensy
