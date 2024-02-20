#include <iostream>

#include <tinympc/admm.hpp>
#include <tinympc/tiny_data_workspace.hpp>

using namespace Eigen;
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

#ifdef __cplusplus
extern "C"
{
#endif

#define NTOTAL 201
#define NRUNS (NTOTAL - NHORIZON - 1)
#define dt 0.05

	float kp = 7.0;
	float kd = 3.0;
	float ki = 0.1;

	Vector3f integral;
	tiny_VectorNx xk;
	tiny_VectorNx xhrz;
	tiny_VectorNx xd;
	tiny_VectorNu uk;

	int main()
	{
		int NPOS = int(NSTATES / 2);
		int exitflag = 1;
		// Double check some data
		// std::cout << tiny_data_solver.settings->max_iter << std::endl;
		// std::cout << tiny_data_solver.cache->AmBKt.format(CleanFmt) << std::endl;
		// std::cout << tiny_data_solver.work->Adyn.format(CleanFmt) << std::endl;
		tiny_data_solver.settings->en_state_bound = 1;
		tiny_data_solver.settings->max_iter = 100;

		for (int i = 0; i < NHORIZON - 1; i++)
		{
			tiny_data_solver.work->x_min.col(i) = -1.0 * tiny_VectorNx::Ones();
			tiny_data_solver.work->x_max.col(i) = 1.0 * tiny_VectorNx::Ones();
			// tiny_data_solver.work->u_min.col(i) << -3, -3, -1;
			// tiny_data_solver.work->u_max.col(i) << 1, 1, 1;
		}

		srand(1);
		xk = tiny_VectorNx::Random() * 0.1;

		for (int step = 0; step < NRUNS; step++)
		{
			xhrz << xk;
			// Rollout the nominal system
			for (int k = 0; k < NHORIZON - 1; k++)
			{
				float temp = 2.0 * sin(1 * dt * step);
				xd(seq(0, NPOS - 1)) = temp * VectorXf::Ones(NPOS);
				// pid controller
				integral = integral + (xd(seq(0, NPOS - 1)) - xhrz(seq(0, NPOS - 1))) * dt;
				tiny_data_solver.work->Uref.col(k) << kp * (xd(seq(0, NPOS - 1)) - xhrz(seq(0, NPOS - 1))) + kd * (xd(seq(NPOS, NSTATES - 1)) - xhrz(seq(NPOS, NSTATES - 1))) + 0 * integral;
				xhrz = tiny_data_solver.work->Adyn * xhrz + tiny_data_solver.work->Bdyn * tiny_data_solver.work->Uref.col(k);
			}
			uk = tiny_data_solver.work->Uref.col(0);

			if (1)
			{ // enable safety filter
				tiny_data_solver.work->x.col(0) << xk;

				exitflag = tiny_solve(&tiny_data_solver);

				uk = tiny_data_solver.work->u.col(0);

				if (exitflag != 1)
					printf("OOPS! Something went wrong!\n");

				// std::cout << tiny_data_solver.work->u.format(CleanFmt) << std::endl;
			}
			xk = tiny_data_solver.work->Adyn * xk + tiny_data_solver.work->Bdyn * uk;
			std::cout << "xk = " << xk.transpose() << std::endl;
			std::cout << "uk = " << uk.transpose() << "\n"
					  << std::endl;
		}
		return 0;
	}

#ifdef __cplusplus
} /* extern "C" */
#endif