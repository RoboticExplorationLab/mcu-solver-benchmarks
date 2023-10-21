#include <iostream>

#include "admm.hpp"
#include "problem_data/rand_prob_tinympc_params.hpp"
#include "problem_data/rand_prob_tinympc_xbar.hpp"

#include "Arduino.h"

// using Eigen::Matrix;

#define DT 1 / 100

extern "C"
{
    static TinyCache cache;
    static TinyWorkspace work;
    static TinySettings settings;
    static TinySolver solver{&settings, &cache, &work};

    void setup()
    {
        srand(123);
        Serial.begin(9600);
        while (!Serial)
        {
            continue;
        }
        Serial.println("Serial initialized");
        // Copy data from problem_data/rand_prob_tinympc_*.hpp
        cache.rho = rho_value;
        cache.Kinf = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
        cache.Pinf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_data);
        cache.Quu_inv = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
        cache.AmBKt = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
        cache.coeff_d2p = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_data);

        work.Adyn = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
        work.Bdyn = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);        
        work.Q = Eigen::Map<tiny_VectorNx>(Q_data);
        work.Qf = Eigen::Map<tiny_VectorNx>(Qf_data);
        work.R = Eigen::Map<tiny_VectorNu>(R_data);
        work.u_min = tiny_MatrixNuNhm1::Constant(-3.0);
        work.u_max = tiny_MatrixNuNhm1::Constant(3.0);
        work.x_min = tiny_MatrixNxNh::Constant(-10000);
        work.x_max = tiny_MatrixNxNh::Constant(10000);
        // for (int i=0; i<NHORIZON; i++) {
        // work.x_min[i] = tiny_VectorNc::Constant(-99999); // Currently unused
        // work.x_max[i] = tiny_VectorNc::Zero();
        // work.x_min[i] = tiny_MatrixNxNh::Constant(-10000);
        // work.x_max[i] = tiny_MatrixNxNh::Constant(10000);
        // work.A_constraints[i] = tiny_MatrixNcNx::Zero();
        // }
        work.Xref = tiny_MatrixNxNh::Zero();
        work.Uref = tiny_MatrixNuNhm1::Zero();
        std::cout << "Adyn: " << work.Adyn << std::endl;
       
        work.x = tiny_MatrixNxNh::Zero();
        work.q = tiny_MatrixNxNh::Zero();
        work.p = tiny_MatrixNxNh::Zero();
        work.v = tiny_MatrixNxNh::Zero();
        work.vnew = tiny_MatrixNxNh::Zero();
        work.g = tiny_MatrixNxNh::Zero();

        work.u = tiny_MatrixNuNhm1::Zero();
        work.r = tiny_MatrixNuNhm1::Zero();
        work.d = tiny_MatrixNuNhm1::Zero();
        work.z = tiny_MatrixNuNhm1::Zero();
        work.znew = tiny_MatrixNuNhm1::Zero();
        work.y = tiny_MatrixNuNhm1::Zero();

        work.primal_residual_state = 0;
        work.primal_residual_input = 0;
        work.dual_residual_state = 0;
        work.dual_residual_input = 0;
        work.status = 0;
        work.iter = 0;

        settings.abs_pri_tol = 0.0001;
        settings.abs_dua_tol = 0.0001;
        settings.max_iter = 4000;
        settings.check_termination = 1;
        settings.en_input_bound = 1;
        settings.en_state_bound = 0;

        std::cout << "Setup complete" << std::endl;


        // Matrix<tinytype, NSTATES, 1> Xref_origin;
        // Xref_origin << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        // // work.Xref = Xref_total.block<NSTATES, NHORIZON>(0,0);
        // work.Xref = Xref_origin.replicate<1,NHORIZON>();
        // // work.x.col(0) = work.Xref.col(0);
        // work.x.col(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        // Xref_total = Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>>(Xref_data).transpose();
        Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>> Xref_total(Xref_data);

        for (int i = 0; i < NTOTAL - NHORIZON; i++)
        {

            // Xref_total = Eigen::Map<Matrix<Eigen::RowMajor, NTOTAL, NSTATES, tinytype>>(Xref_data).transpose();
            work.Xref.block<NSTATES, NHORIZON>(0, 0) = Xref_total.transpose().block<NSTATES, NHORIZON>(0, i);

            // Serial.print(availableMemory());
            unsigned long start = micros();
            tiny_solve(&solver);
            unsigned long end = micros();
            printf("%10.4f %10d %10d\n", (work.x.col(0) - work.Xref.col(0)).squaredNorm(), work.iter, end - start);
            // // Serial.println(end-start);
            // // std::cout << "TIMING: " << end-start << std::endl;
            // // Serial.println((work.x.col(0) - work.Xref.col(0)).squaredNorm());
            // // std::cout << "SQUARED DIFFERENCE: " << (work.x.col(0) - work.Xref.col(0)).squaredNorm() << std::endl;

            tiny_VectorNx random_num;
            for (int j = 0; j < NSTATES; ++j)
            {
                random_num(j) = (float(rand() / RAND_MAX) - 0.5)*2 * 0.01;
            }
            tiny_VectorNx next_x = work.Adyn * work.x.col(0) + work.Bdyn * work.u.col(0);
            work.x.col(0) = next_x + random_num;
        }
    }
}

void loop()
{
}