#include <iostream>
#include "src/tinympc/admm.hpp"
#include "src/tinympc/tiny_data_workspace.hpp"
#include "Arduino.h"
using namespace Eigen;

// IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

extern "C"
{

    #define NTOTAL 201
    #define NRUNS (NTOTAL - NHORIZON - 1)
    #define dt 0.05
    #define NPOS (NSTATES / 2)

	float kp = 7.0;
	float kd = 3.0;
	float ki = 0.1;

	tiny_VectorNx xk;
	tiny_VectorNx xhrz;
	tiny_VectorNx xd;
	tiny_VectorNu uk;
	
	static int i = 0;

	void setup()
	{
		Serial.begin(9600);
		delay(5000);
		Serial.println("Serial initialized");
    Serial.println("Start TinyMPC Safety Filter");

		int exitflag = 1;
		// Double check some data
		// std::cout << tiny_data_solver.settings->max_iter << std::endl;
		// std::cout << tiny_data_solver.cache->AmBKt.format(CleanFmt) << std::endl;
		// std::cout << tiny_data_solver.work->Adyn.format(CleanFmt) << std::endl;
		tiny_data_solver.settings->en_state_bound = 1;
		tiny_data_solver.settings->max_iter = 500;

		for (int i = 0; i < NHORIZON - 1; i++)
		{
			// tiny_data_solver.work->x_min.col(i) = -1.5 * tiny_VectorNx::Ones();
			// tiny_data_solver.work->x_max.col(i) = 1.5 * tiny_VectorNx::Ones();
			// tiny_data_solver.work->u_min.col(i) = -2.0 * tiny_VectorNu::Ones();
			// tiny_data_solver.work->u_max.col(i) = 2.0 * tiny_VectorNu::Ones();
		}

		srand(1);
		xk = tiny_VectorNx::Random() * 0.1;
		// xk << 0.0, 0.0, 0.0, 0.0, 0, 0;

		for (int step = 0; step < NRUNS; step++)
		{
			xhrz = xk;
			// Rollout the nominal system
			for (int k = 0; k < NHORIZON - 1; k++)
			{
				float temp = 2.0 * sin(1 * dt * (step + k));
				xd(seq(0, NPOS - 1)) = temp * VectorXf::Ones(NPOS);
				// pid controller
				tiny_data_solver.work->Uref.col(k) = kp * (xd(seq(0, NPOS - 1)) - xhrz(seq(0, NPOS - 1))) + kd * (xd(seq(NPOS, NSTATES - 1)) - xhrz(seq(NPOS, NSTATES - 1)));
				xhrz = tiny_data_solver.work->Adyn * xhrz + tiny_data_solver.work->Bdyn * tiny_data_solver.work->Uref.col(k);
			}
			uk = tiny_data_solver.work->Uref.col(0);

			if (1)
			{ // enable safety filter
				tiny_data_solver.work->x.col(0) = xk;

				unsigned long start = micros();
				exitflag = tiny_solve(&tiny_data_solver);
				unsigned long end = micros();

				// BENCHMARKING DATA
				printf("%10d %10.6d\n", tiny_data_solver.work->iter, end - start);
				// Serial.print(abs((tiny_data_solver.work->x.col(1) - tiny_data_solver.work->Xref.col(1)).norm()));
				Serial.print(" "); Serial.print(tiny_data_solver.work->iter); Serial.print(" ");
				Serial.println(end-start);

				uk = tiny_data_solver.work->u.col(0);

				// if (exitflag != 1)
				// 	printf("OOPS! Something went wrong!\n");

				// std::cout << tiny_data_solver.work->u.format(CleanFmt) << std::endl;
			}
			xk = tiny_data_solver.work->Adyn * xk + tiny_data_solver.work->Bdyn * uk;
			// std::cout << "xk = " << xk.transpose() << std::endl;
			// std::cout << "uk = " << uk.transpose() << "\n"
			// 		  << std::endl;
		}

	}
	
	void loop()
	{
		// Serial.println(i++);
		// delay(1000);
	}

} /* extern "C" */