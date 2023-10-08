#include <iostream>

#include "admm.hpp"
#include "problem_data/rand_prob_tinympc_params.hpp"
#include "problem_data/rand_prob_tinympc_xbar.hpp"

#include "Arduino.h"

using Eigen::Matrix;

#define DT 1 / 100

extern "C"
{
    static struct tiny_cache cache;
    static struct tiny_params params;
    static struct tiny_problem problem;
    // static Eigen::Matrix<tinytype, NSTATES, NTOTAL, Eigen::ColMajor> Xref_total;

    void setup()
    {
        srand(123);
        Serial.begin(9600);
        while (!Serial)
        {
            continue;
        }

        // Copy data from problem_data/quadrotor*.hpp
        
        cache.Adyn[0] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
        cache.Bdyn[0] = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);
        cache.rho[0] = rho_value;
        cache.Kinf[0] = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
        cache.Pinf[0] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_data);
        cache.Quu_inv[0] = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
        cache.AmBKt[0] = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
        cache.coeff_d2p[0] = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_data);

        
        params.Q[0] = Eigen::Map<tiny_VectorNx>(Q_data);
        params.Qf[0] = Eigen::Map<tiny_VectorNx>(Qf_data);
        params.R[0] = Eigen::Map<tiny_VectorNu>(R_data);
        params.Q[1] = Eigen::Map<tiny_VectorNx>(Q_data);
        params.Qf[1] = Eigen::Map<tiny_VectorNx>(Qf_data);
        params.R[1] = Eigen::Map<tiny_VectorNu>(R_data);
        params.u_min = tiny_MatrixNuNhm1::Constant(-3.0);
        params.u_max = tiny_MatrixNuNhm1::Constant(3.0);
        params.x_min = tiny_MatrixNxNh::Constant(-10000);
        params.x_max = tiny_MatrixNxNh::Constant(10000);
        // for (int i=0; i<NHORIZON; i++) {
        // params.x_min[i] = tiny_VectorNc::Constant(-99999); // Currently unused
        // params.x_max[i] = tiny_VectorNc::Zero();
        // params.x_min[i] = tiny_MatrixNxNh::Constant(-10000);
        // params.x_max[i] = tiny_MatrixNxNh::Constant(10000);
        // params.A_constraints[i] = tiny_MatrixNcNx::Zero();
        // }
        params.Xref = tiny_MatrixNxNh::Zero();
        params.Uref = tiny_MatrixNuNhm1::Zero();
        params.cache = cache;

       
        problem.x = tiny_MatrixNxNh::Zero();
        problem.q = tiny_MatrixNxNh::Zero();
        problem.p = tiny_MatrixNxNh::Zero();
        problem.v = tiny_MatrixNxNh::Zero();
        problem.vnew = tiny_MatrixNxNh::Zero();
        problem.g = tiny_MatrixNxNh::Zero();

        problem.u = tiny_MatrixNuNhm1::Zero();
        problem.r = tiny_MatrixNuNhm1::Zero();
        problem.d = tiny_MatrixNuNhm1::Zero();
        problem.z = tiny_MatrixNuNhm1::Zero();
        problem.znew = tiny_MatrixNuNhm1::Zero();
        problem.y = tiny_MatrixNuNhm1::Zero();

        problem.primal_residual_state = 0;
        problem.primal_residual_input = 0;
        problem.dual_residual_state = 0;
        problem.dual_residual_input = 0;
        problem.abs_tol = 0.0001;
        problem.status = 0;
        problem.iter = 0;
        problem.max_iter = 4000;
        problem.iters_check_rho_update = 10;
        problem.cache_level = 0;

        // Matrix<tinytype, NSTATES, 1> Xref_origin;
        // Xref_origin << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        // // params.Xref = Xref_total.block<NSTATES, NHORIZON>(0,0);
        // params.Xref = Xref_origin.replicate<1,NHORIZON>();
        // // problem.x.col(0) = params.Xref.col(0);
        // problem.x.col(0) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        // Xref_total = Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>>(Xref_data).transpose();
        Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>> Xref_total(Xref_data);

        for (int i = 0; i < NTOTAL - NHORIZON; i++)
        {

            // Xref_total = Eigen::Map<Matrix<Eigen::RowMajor, NTOTAL, NSTATES, tinytype>>(Xref_data).transpose();
            params.Xref.block<NSTATES, NHORIZON>(0, 0) = Xref_total.transpose().block<NSTATES, NHORIZON>(0, i);
            // std::cout << params.Xref << std::endl;
            // Serial.println(problem.u.col(0)(0));
            // Serial.println(problem.u.col(0)(1));
            // Serial.println(problem.u.col(0)(2));
            // Serial.println(problem.u.col(0)(3));

            // Serial.print(availableMemory());
            unsigned long start = micros();
            solve_admm(&problem, &params);
            unsigned long end = micros();
            printf("%10.4f %10d %10d\n", (problem.x.col(0) - params.Xref.col(0)).squaredNorm(), problem.iter, end - start);
            // Serial.println(end-start);
            // std::cout << "TIMING: " << end-start << std::endl;
            // Serial.println((problem.x.col(0) - params.Xref.col(0)).squaredNorm());
            // std::cout << "SQUARED DIFFERENCE: " << (problem.x.col(0) - params.Xref.col(0)).squaredNorm() << std::endl;
            // Serial.println(problem.x.col(0)(0));
            // Serial.println(params.Xref.col(0)(0));
            // Serial.println('\n');

            // Serial.println(problem.x.col(0)(1));
            // Serial.println(params.Xref.col(0)(1));
            // Serial.println('\n');

            // Serial.println(problem.x.col(0)(1));
            // Serial.println(params.Xref.col(0)(1));
            // Serial.println('\n');

            // Serial.println(problem.x.col(0)(0));
            // Serial.println(problem.x.col(0)(1));
            // Serial.println(problem.x.col(0)(2));

            // Serial.println(params.Xref.col(0)(0));
            // Serial.println(params.Xref.col(0)(1));
            // Serial.println(params.Xref.col(0)(2));

            // Serial.println(problem.u.col(0)(0));
            // Serial.println(problem.u.col(0)(1));
            // Serial.println(problem.u.col(0)(2));
            // Serial.println(problem.u.col(0)(3));
            // std::cout << problem.u.col(0)(0);

            // std::cout << problem.x.col(0)(0) << std::endl;
            // std::cout << problem.x.col(0)(1) << std::endl;
            // std::cout << problem.x.col(0)(2) << std::endl;
            // std::cout << params.Xref.col(0)(0) << std::endl;
            // std::cout << params.Xref.col(0)(1) << std::endl;
            // std::cout << params.Xref.col(0)(2) << std::endl;
            // std::cout << problem.u.col(0)(0) << std::endl;
            // std::cout << problem.u.col(0)(1) << std::endl;
            // std::cout << problem.u.col(0)(2) << std::endl;
            // std::cout << problem.u.col(0)(3) << std::endl;

            tiny_VectorNx random_num;
            for (int j = 0; j < NSTATES; ++j)
            {
                random_num(j) = (float(rand() / RAND_MAX) - 0.5)*2 * 0.01;
            }
            tiny_VectorNx next_x = cache.Adyn[0] * problem.x.col(0) + cache.Bdyn[0] * problem.u.col(0);
            problem.x.col(0) = next_x + random_num;
        }
    }
}

void loop()
{
}