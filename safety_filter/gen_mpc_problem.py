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

def osqp_export_data_to_c(A, B, R, NSTATES, NINPUTS, NHORIZON, NTOTAL):
    SIZE_Q = (NHORIZON) * NSTATES + (NHORIZON-1) * NINPUTS
    SIZE_LU = (NHORIZON) * NSTATES * 2 + (NHORIZON-1) * NINPUTS

    include_statement = '#include "osqp_api_types.h"\n\n'
    boilerplate = "#pragma once\n\n"

    # f = open("rand_prob_osqp_xbar.h", "w")
    f = open('osqp_teensy/src'+"/osqp_problem.h", "w")
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