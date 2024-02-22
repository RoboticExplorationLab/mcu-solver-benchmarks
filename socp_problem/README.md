# SOCP-Based Rocket Soft Landing

Generated code for ECOS and SCS is in `rocket_landing`. They use the same CPG interface so we use the same `cpg_example.c` program for both but noticing the options.
SCS works perfectly while ECOS cannot be updated properly for MPC. Therefore, we have to codegen ECOS again and again for each step (cubersome).
See *QP-Based Predictive Safety Filtering* for automatic pipeline. You need to move these source code to Teensy.

Run `rocket_landing/gen_rocket.py` to select solver and generate C code. They will use the same main program `cpg_example.c` found in this dir.
