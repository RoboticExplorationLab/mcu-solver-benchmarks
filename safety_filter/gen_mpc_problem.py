"""
File: prob_data_gen.py
Author: Anoushka, Khai
Date: 2023-09-15
Description: A Python script to generate random MPC problem for OSQP (Python) and TinyMPC (C++).
"""

import numpy as np
import os

def export_xref_to_c(declare, data, NSTATES):
    string = declare + "= {\n"
    for i in range(data.shape[0]):
        string = string + "  "
        for j in range(NSTATES):
            if i == data.shape[0] and j == NSTATES:
                this_str = str(data[i][j]) + ",\t"
            else:
                this_str = str(data[i][j]) + ",\t"
            # str = str * this_str * "f"
            string = string + this_str
        string = string + "\n"
    string = string + "};"
    return string

def export_mat_to_c(declare, data):
    string = declare + "= {\n"
    for i in range(data.shape[0]):
        string = string + "  "
        for j in range(data.shape[1]):
            if i == data.shape[0] and j == data.shape[1]:
                this_str = str(data[i][j]) + ",\t"
            else:
                 this_str = str(data[i][j]) + ",\t"
            string = string + this_str
        string = string + "\n"
    string = string + "};"
    return string

def tinympc_export_data_to_c(xbar, A, B, Q, Qf, R, umin, umax, xmin, xmax, NSTATES, NINPUTS, NHORIZON, NTOTAL):
    include_statement = '#include "types.hpp"\n\n'
    boilerplate = "#pragma once\n\n"

    xbar_string = export_xref_to_c("const PROGMEM tinytype Xref_data["+str(NTOTAL)+"*"+str(NSTATES)+"] ", xbar, NSTATES)

    A_data_string = export_mat_to_c("const PROGMEM tinytype Adyn_data["+str(NSTATES)+"*"+str(NSTATES)+"] ", A) + "\n\n"
    B_data_string = export_mat_to_c("const PROGMEM tinytype Bdyn_data["+str(NSTATES)+"*"+str(NINPUTS)+"] ", B) + "\n\n"
   
    Q_diag = np.diagonal(Q)
    Q_data_string = "const PROGMEM tinytype Q_data["+str(NSTATES)+"] = {"
    for i in range(NSTATES):
        Q_data_string = Q_data_string + str(Q_diag[i])
        if i != NSTATES-1:
            Q_data_string = Q_data_string + ','
    Q_data_string = Q_data_string+"};\n\n"
    
    Qf_diag = np.diagonal(Qf)
    Qf_data_string = "const PROGMEM tinytype Qf_data["+str(NSTATES)+"] = {"
    for i in range(NSTATES):
        Qf_data_string = Qf_data_string + str(Qf_diag[i])
        if i != NSTATES-1:
            Qf_data_string = Qf_data_string + ','
    Qf_data_string = Qf_data_string+"};\n\n"
    
    R_diag = np.diagonal(R)
    R_data_string = "const PROGMEM tinytype R_data["+str(NINPUTS)+"] = {"
    for i in range(NINPUTS):
        R_data_string = R_data_string + str(R_diag[i])
        if i != NINPUTS-1:
            R_data_string = R_data_string + ','
    R_data_string = R_data_string+"};\n\n"

    umin_string = export_mat_to_c("const PROGMEM tinytype umin["+str(NINPUTS)+"] ", umin) + "\n\n"
    umax_string = export_mat_to_c("const PROGMEM tinytype umax["+str(NINPUTS)+"] ", umax) + "\n\n"
    xmin_string = export_mat_to_c("const PROGMEM tinytype xmin["+str(NSTATES)+"] ", xmin) + "\n\n"
    xmax_string = export_mat_to_c("const PROGMEM tinytype xmax["+str(NSTATES)+"] ", xmax) + "\n\n"

    f = open('random_problems/prob_nx_'+str(NSTATES)+'/constants.hpp','w')
    f.write("#define NSTATES "+str(NSTATES)+'\n\n')
    f.write("#define NINPUTS "+str(NINPUTS)+'\n\n')
    f.write("#define NHORIZON "+str(NHORIZON)+'\n\n')
    f.write("#define NTOTAL "+str(NTOTAL)+'\n\n')
    f.write('#define NSTATE_CONSTRAINTS 1\n\n')

    f.close()

    f = open('random_problems/prob_nx_'+str(NSTATES)+"/rand_prob_tinympc_xbar.hpp", "w")
    # f = open("rand_prob_tinympc_xbar.hpp", "w")
    f.write(include_statement)
    f.write(boilerplate)
    f.write(xbar_string)
    f.close()

    f = open('random_problems/prob_nx_'+str(NSTATES)+"/rand_prob_tinympc_params.hpp", "w")
    # f = open("rand_prob_tinympc_params.hpp", "w")
    f.write(include_statement)
    f.write(boilerplate)
    f.write(A_data_string)
    f.write(B_data_string)
    f.write(Q_data_string)
    f.write(Qf_data_string)
    f.write(R_data_string)
    f.write(umin_string)
    f.write(umax_string)
    f.write(xmin_string)
    f.write(xmax_string)
        
    K = np.zeros((NINPUTS,NSTATES))
    P = np.zeros((NSTATES,NSTATES))
    Kprev = np.zeros((NINPUTS,NSTATES))
    Pprev = np.zeros((NSTATES,NSTATES))

    # Compute Kinf, Pinf
    riccati_iters = 0
    riccati_err = 1e-10
    Pprev = Qf
    while True:
        K = np.linalg.inv(R + B.T @ Pprev @ B) @ (B.T @ Pprev @ A)
        P = Q + A.T @ Pprev @ (A - B@K)
        if np.max(np.abs(K - Kprev)) < riccati_err:
            break
        Kprev = K
        Pprev = P
        riccati_iters += 1

    # Cache precomputed values
    rho = 0.1
    Q = Q + rho*np.eye(NSTATES)
    Qf = P + rho*np.eye(NSTATES)
    R = R + rho*np.eye(NINPUTS)

    # Compute Kinf, Pinf
    riccati_iters = 0
    riccati_err = 1e-10
    Pprev = Qf
    while True:
        K = np.linalg.inv(R + B.T @ Pprev @ B) @ (B.T @ Pprev @ A)
        P = Q + A.T @ Pprev @ (A - B@K)
        if np.max(np.abs(K - Kprev)) < riccati_err:
            break
        Kprev = K
        Pprev = P
        riccati_iters += 1

    Kinf = K
    Pinf = P
    Quu_inv = np.linalg.inv(R + B.T @ Pinf@B)
    AmBKt = (A - B@K).T
    coeff_d2p_list = Kinf.T@R - AmBKt@Pinf@B

    rho_string = "const PROGMEM tinytype rho_value = "+str(rho)+";\n\n"
    Kinf_string = export_mat_to_c("const PROGMEM tinytype Kinf_data["+str(NINPUTS)+"*"+str(NSTATES)+"] ", Kinf) + "\n\n"
    Pinf_string = export_mat_to_c("const PROGMEM tinytype Pinf_data["+str(NSTATES)+"*"+str(NSTATES)+"] ", Pinf) + "\n\n"
    Quu_inv_string = export_mat_to_c("const PROGMEM tinytype Quu_inv_data["+str(NINPUTS)+"*"+str(NINPUTS)+"] ", Quu_inv) + "\n\n"
    AmBKt_string = export_mat_to_c("const PROGMEM tinytype AmBKt_data["+str(NSTATES)+"*"+str(NSTATES)+"] ", AmBKt) + "\n\n"
    coeff_d2p_list_string = export_mat_to_c("const PROGMEM tinytype coeff_d2p_data["+str(NSTATES)+"*"+str(NINPUTS)+"] ", coeff_d2p_list) + "\n\n"

    f.write(rho_string)
    f.write(Kinf_string)
    f.write(Pinf_string)
    f.write(Quu_inv_string)
    f.write(AmBKt_string)
    f.write(coeff_d2p_list_string)
    f.close()

def osqp_export_data_to_c(A, B, R, NSTATES, NINPUTS, NHORIZON, NTOTAL):
    SIZE_Q = (NHORIZON) * NSTATES + (NHORIZON-1) * NINPUTS
    SIZE_LU = (NHORIZON) * NSTATES * 2 + (NHORIZON-1) * NINPUTS

    include_statement = '#include "osqp_api_types.h"\n\n'
    boilerplate = "#pragma once\n\n"

    # f = open("rand_prob_osqp_xbar.h", "w")
    f = open('osqp_teensy/lib/osqp/inc/public'+"/osqp_problem.h", "w")
    f.write(include_statement)
    f.write(boilerplate)

    f.write("#define NSTATES "+str(NSTATES)+'\n\n')
    f.write("#define NINPUTS "+str(NINPUTS)+'\n\n')
    f.write("#define NHORIZON "+str(NHORIZON)+'\n\n')
    f.write("#define NTOTAL "+str(NTOTAL)+'\n\n')
    f.write("#define SIZE_Q "+str(SIZE_Q)+'\n\n')
    f.write("#define SIZE_LU "+str(SIZE_LU)+'\n\n')

    mR_string = export_mat_to_c("const PROGMEM OSQPFloat mR["+str(NSTATES)+"*"+str(NSTATES)+"] ", -R)+'\n\n'
    A_string = export_mat_to_c("const PROGMEM OSQPFloat A["+str(NSTATES)+"*"+str(NSTATES)+"] ", A)+'\n\n'
    B_string = export_mat_to_c("const PROGMEM OSQPFloat B["+str(NSTATES)+"*"+str(NINPUTS)+"] ", B)+'\n\n'

    f.write(mR_string)
    f.write(A_string)
    f.write(B_string)

    f.close()

def generate_data(NSTATES, NINPUTS, NHORIZON, NTOTAL):
    np.random.seed(123)
    # Generate Q: Q_{ii} = U(0, 1)
    Q_diag = np.random.uniform(0,10,NSTATES)
    Q = np.diag(Q_diag)

    # Generate R: R_{ii} = 0.1
    R_diag = 0.1*np.ones(NINPUTS)
    R = np.diag(R_diag)

    # Generate Qf: (N-1)*Q
    Qf = ((NHORIZON)-1)*Q

    # reference trajectory xk, uk --> uk ~ N(0, 1), xk+1 = f(xk, uk)
    u = np.random.normal(size=(NINPUTS, NTOTAL-1))

    # A and B are random matrices with eigenvalues inside of the unit circle

    # Generate a controllable system
    while True:
        # SVD for both A and B to ensure that the eigenvalues are inside of the unit circle
        A = np.random.uniform(low=-1, high=1, size=(NSTATES, NSTATES))
        U, S, Vh = np.linalg.svd(A) # S is a vector of the non-zero singular values
        E = np.zeros((U.shape[0], Vh.shape[0])) # E is the SVD matrix of singular values Σ 
        S = S / np.max(S) # Scale the singular values so that 
        np.fill_diagonal(E, S)
        A = U @ E @ Vh.T

        B = np.random.uniform(low=-1, high=1, size=(NSTATES,NINPUTS))

        # Check if the system is controllable
        C = np.zeros((NSTATES,NSTATES*NINPUTS)) # controllability matrix
        Ak = np.zeros((NSTATES,NSTATES)) + np.eye(NSTATES)
        for k in range(NSTATES):
            C[:, (k)*NINPUTS:(k)*NINPUTS+NINPUTS] = Ak@B
            Ak = Ak @ A

        if np.linalg.matrix_rank(C) == A.shape[0]: # only true if system is controllable
            break
        else:
            print("Not controllable")

    umax = np.zeros((NINPUTS,1)) + 3
    umin = np.zeros((NINPUTS,1)) - 3
    xmin = np.zeros((NSTATES,1)) - 10000 # might add state constraints later
    xmax = np.zeros((NSTATES,1)) + 10000

    x0 = np.zeros(NSTATES)
    xbar = np.zeros((NTOTAL, NSTATES))
    xbar[0,:] = x0

    for k in range(NTOTAL-1):
        xbar[k+1,:] = A @ xbar[k,:] + B @ u[:,k]


    ### SAVE ALL RANDOM PROBLEM DATA TO A HPP FILE TO BE USED BY ALL SOLVERS ###

    tinympc_export_data_to_c(xbar, A, B, Q, Qf, R, umin, umax, xmin, xmax, NSTATES, NINPUTS, NHORIZON, NTOTAL)
    osqp_export_data_to_python(xbar, A, B, Q, Qf, R, umin, umax, xmin, xmax, NSTATES, NINPUTS, NHORIZON, NTOTAL)
        
if __name__ == '__main__':
    ## Vary NHORIZON
    # NSTATES = 10
    # NINPUTS = 4
    # NHORIZON = 10
    # NTOTAL = 200

    # for NHORIZON in [8, 10, 12, 14, 16, 25, 50, 75, 100]:
    #     os.system('mkdir random_problems/prob_Nh_'+str(NHORIZON))
    #     generate_data(NSTATES, NINPUTS, NHORIZON, NTOTAL)

    ## Vary NSTATES
    NINPUTS = 4
    NHORIZON = 10
    NTOTAL = 200

    for NSTATES in [10]:
        os.system('mkdir random_problems/prob_nx_'+str(NSTATES))
        generate_data(NSTATES, NINPUTS, NHORIZON, NTOTAL)
        print('generated: random_problems/prob_nx_'+str(NSTATES))

    # # ## Vary NINPUTS
    # NSTATES = 10
    # NINPUTS = 4
    # NHORIZON = 10
    # NTOTAL = 200

    # for NINPUTS in range(2,30,2):
    #     os.system('mkdir random_problems/prob_nu_'+str(NINPUTS))
    #     generate_data(NSTATES, NINPUTS, NHORIZON, NTOTAL)
