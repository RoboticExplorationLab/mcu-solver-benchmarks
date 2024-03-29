
/*
Auto-generated by CVXPYgen on February 26, 2024 at 17:05:22.
Content: Declarations for Python binding with pybind11.
*/

// User-defined parameters
struct CPG_Params_cpp_t {
    std::array<double, 285> param3;
    std::array<double, 6> param1;
};

// Flags for updated user-defined parameters
struct CPG_Updated_cpp_t {
    bool param3;
    bool param1;
};

// Primal solution
struct CPG_Prim_cpp_t {
    std::array<double, 285> var2;
};

// Dual solution
struct CPG_Dual_cpp_t {
    std::array<double, 6> d0;
    std::array<double, 6> d1;
    std::array<double, 6> d2;
    std::array<double, 6> d3;
    std::array<double, 6> d4;
    std::array<double, 6> d5;
    std::array<double, 6> d6;
    std::array<double, 6> d7;
    std::array<double, 6> d8;
    std::array<double, 6> d9;
    std::array<double, 6> d10;
    std::array<double, 6> d11;
    std::array<double, 6> d12;
    std::array<double, 6> d13;
    std::array<double, 6> d14;
    std::array<double, 6> d15;
    std::array<double, 6> d16;
    std::array<double, 6> d17;
    std::array<double, 6> d18;
    std::array<double, 6> d19;
    std::array<double, 6> d20;
    std::array<double, 6> d21;
    std::array<double, 6> d22;
    std::array<double, 6> d23;
    std::array<double, 6> d24;
    std::array<double, 6> d25;
    std::array<double, 6> d26;
    std::array<double, 6> d27;
    std::array<double, 6> d28;
    std::array<double, 6> d29;
    std::array<double, 6> d30;
    std::array<double, 6> d31;
    double d32;
    double d33;
    double d34;
    double d35;
    double d36;
    double d37;
    double d38;
    double d39;
    double d40;
    double d41;
    double d42;
    double d43;
    double d44;
    double d45;
    double d46;
    double d47;
    double d48;
    double d49;
    double d50;
    double d51;
    double d52;
    double d53;
    double d54;
    double d55;
    double d56;
    double d57;
    double d58;
    double d59;
    double d60;
    double d61;
    double d62;
    std::array<double, 3> d63;
    std::array<double, 3> d64;
    std::array<double, 3> d65;
    std::array<double, 3> d66;
    std::array<double, 3> d67;
    std::array<double, 3> d68;
    std::array<double, 3> d69;
    std::array<double, 3> d70;
    std::array<double, 3> d71;
    std::array<double, 3> d72;
    std::array<double, 3> d73;
    std::array<double, 3> d74;
    std::array<double, 3> d75;
    std::array<double, 3> d76;
    std::array<double, 3> d77;
    std::array<double, 3> d78;
    std::array<double, 3> d79;
    std::array<double, 3> d80;
    std::array<double, 3> d81;
    std::array<double, 3> d82;
    std::array<double, 3> d83;
    std::array<double, 3> d84;
    std::array<double, 3> d85;
    std::array<double, 3> d86;
    std::array<double, 3> d87;
    std::array<double, 3> d88;
    std::array<double, 3> d89;
    std::array<double, 3> d90;
    std::array<double, 3> d91;
    std::array<double, 3> d92;
    std::array<double, 3> d93;
    std::array<double, 3> d94;
    std::array<double, 3> d95;
    std::array<double, 3> d96;
    std::array<double, 3> d97;
    std::array<double, 3> d98;
    std::array<double, 3> d99;
    std::array<double, 3> d100;
    std::array<double, 3> d101;
    std::array<double, 3> d102;
    std::array<double, 3> d103;
    std::array<double, 3> d104;
    std::array<double, 3> d105;
    std::array<double, 3> d106;
    std::array<double, 3> d107;
    std::array<double, 3> d108;
    std::array<double, 3> d109;
    std::array<double, 3> d110;
    std::array<double, 3> d111;
    std::array<double, 3> d112;
    std::array<double, 3> d113;
    std::array<double, 3> d114;
    std::array<double, 3> d115;
    std::array<double, 3> d116;
    std::array<double, 3> d117;
    std::array<double, 3> d118;
    std::array<double, 3> d119;
    std::array<double, 3> d120;
    std::array<double, 3> d121;
    std::array<double, 3> d122;
    std::array<double, 3> d123;
    std::array<double, 3> d124;
};

// Solver information
struct CPG_Info_cpp_t {
    double obj_val;
    int iter;
    int status;
    double pri_res;
    double dua_res;
    double time;
};

// Solution and solver information
struct CPG_Result_cpp_t {
    CPG_Prim_cpp_t prim;
    CPG_Dual_cpp_t dual;
    CPG_Info_cpp_t info;
};

// Main solve function
CPG_Result_cpp_t solve_cpp(struct CPG_Updated_cpp_t& CPG_Updated_cpp, struct CPG_Params_cpp_t& CPG_Params_cpp);
