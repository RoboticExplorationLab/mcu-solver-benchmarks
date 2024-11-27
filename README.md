# mcu-solver-benchmarks

Benchmarking solvers on microcontrollers (MCU), within TinyMPC project

## Installation

* Python >= 3.9
* [OSQP v1 from source](https://github.com/osqp/osqp)
* [CVXPYGEN](https://github.com/cvxgrp/cvxpygen)
* [CVXGEN](https://cvxgen.com/docs/index.html)
* [PlatformIO](https://platformio.org/) as VSCode extension

## How to use

Please check README.md in each folder

## Ready-to-run program

* Problem
  * Random QP-MPC with input constraints
  * Safety filter
  * Rocket landing
* Solver
  * HPIPM (safety filter/random QP-MPC)
  * OSQP (safety filter/random QP-MPC)
  * ECOS (all problems)
  * SCS (all problems)
  * TinyMPC (all problems)
* MCU
  * STM32 (Adafruit feather board)
  * Teensy
