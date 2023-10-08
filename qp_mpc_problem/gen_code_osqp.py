"""
File: prob_data_gen.py
Author: Anoushka, Khai
Date: 2023-09-15
Description: A Python script to generate C code for OSQP from problem data. Use OSQP v1
    Code is stored at `generated_osqp_solver` dir
"""

import osqp
import numpy as np
from scipy import sparse

def generate_osqp_solver(path):
    # Discrete time model of a quadcopter
    data = np.load(path+'/rand_prob_osqp_params.npz')
    nx = data['nx']
    nu = data['nu']
    Nh = data['Nh']
    N = Nh
    Nsim = data['Nsim']
    Q = data['Q']
    R = data['R']
    Ad = sparse.csc_matrix(data['A'])
    Bd = sparse.csc_matrix(data['B'])
    QN = data['Qf']
    xbar_full = data['x_bar'].T
    umin = data['umin'][:,0]
    umax = data['umax'][:,0]
    xmin = data['xmin'][:,0]
    xmax = data['xmax'][:,0]

    x0 = np.zeros(nx)

    # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    # - quadratic objective
    P = sparse.block_diag([sparse.kron(sparse.eye(Nh), Q), QN,
                        sparse.kron(sparse.eye(Nh), R)], format='csc')
    # - linear objective
    q = np.hstack([np.hstack([-Q@xbar_full[:,i] for i in range(Nh)]), -QN@xbar_full[:,Nh], np.zeros(Nh*nu)])
    # - linear dynamics
    Ax = sparse.kron(sparse.eye(Nh+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(Nh+1, k=-1), Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, Nh)), sparse.eye(Nh)]), Bd)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-x0, np.zeros(Nh*nx)])
    ueq = leq

    # - input and state constraints
    Aineq = sparse.eye((Nh+1)*nx + Nh*nu)
    lineq = np.hstack([np.kron(np.ones(Nh+1), xmin), np.kron(np.ones(Nh), umin)])
    uineq = np.hstack([np.kron(np.ones(Nh+1), xmax), np.kron(np.ones(Nh), umax)])
    # - OSQP constraints
    A = sparse.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace and change alpha parameter
    prob.setup(P, q, A, l, u, alpha=1.0, scaling=0, check_termination=1, eps_abs=1e-3, eps_rel=1e-3, eps_prim_inf=1e-4, eps_dual_inf=1e-4, max_iter=4000, polish=False, rho=0.1, adaptive_rho=False, warm_start=True)
    # prob.setup(P, q, A, l, u, warm_starting=True, polish=True)

    # Generate C code
    # fmt: off
    prob.codegen(
        path+'/generated_osqp_solver',   # Output folder for auto-generated code
        prefix='osqp_data_',         # Prefix for filenames and C variables; useful if generating multiple problems
        force_rewrite=True,        # Force rewrite if output folder exists?
        parameters='vectors',      # What do we wish to update in the generated code?
                                   # One of 'vectors' (allowing update of q/l/u through prob.update_data_vec)
                                   # or 'matrices' (allowing update of P/A/q/l/u
                                   # through prob.update_data_vec or prob.update_data_mat)
        use_float=True,
        printing_enable=False,     # Enable solver printing?
        profiling_enable=False,    # Enable solver profiling?
        interrupt_enable=False,    # Enable user interrupt (Ctrl-C)?
        include_codegen_src=True,  # Include headers/sources/Makefile in the output folder,
                                   # creating a self-contained compilable folder?
        extension_name='pyosqp',   # Name of the generated python extension; generates a setup.py; Set None to skip
        compile=False,             # Compile the above python extension into an importable module
                                   # (allowing "import pyosqp")?
    )
    # fmt: on
