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

def socp_export_data_to_c(dir, A, B, fdyn, Q_single, NSTATES, NINPUTS, NHORIZON, NTOTAL):
    SIZE_Q = (NHORIZON) * NSTATES + (NHORIZON-1) * NINPUTS
    SIZE_LU = (NHORIZON) * NSTATES * 2 + (NHORIZON-1) * NINPUTS

    boilerplate = "#pragma once\n\n"
    include_statement = '#include "cpg_workspace.h"\n\n'

    f = open(dir+"/cpg_problem.h", "w")

    f.write(boilerplate)
    f.write(include_statement)

    f.write("#define NSTATES "+str(NSTATES)+'\n\n')
    f.write("#define NINPUTS "+str(NINPUTS)+'\n\n')
    f.write("#define NHORIZON "+str(NHORIZON)+'\n\n')
    f.write("#define NTOTAL "+str(NTOTAL)+'\n\n')
    f.write("#define Q_single "+str(Q_single)+'\n\n')

    A_string = export_mat_to_c("const PROGMEM cpg_float A["+str(NSTATES)+"*"+str(NSTATES)+"] ", A)+'\n\n'
    B_string = export_mat_to_c("const PROGMEM cpg_float B["+str(NSTATES)+"*"+str(NINPUTS)+"] ", B)+'\n\n'
    f_string = export_mat_to_c("const PROGMEM cpg_float f["+str(NSTATES)+"] ", fdyn.reshape(NSTATES,1))+'\n\n'

    f.write(A_string)
    f.write(B_string)
    f.write(f_string)

    f.close()

def replace_in_file(file_path, old_lines, new_lines):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    with open(file_path, 'w') as file:
        for line in lines:
            if line.strip() in old_lines:
                file.write(new_lines[old_lines.index(line.strip())] + '\n')
            else:
                file.write(line)

