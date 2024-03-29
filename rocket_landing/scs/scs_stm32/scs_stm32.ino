
/*
Auto-generated by CVXPYgen on February 16, 2024 at 16:59:26.
Content: Example program for updating parameters, solving, and inspecting the result.
*/

#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "time.h"

#include "Arduino.h"

extern "C" {

#include "src/cpg_workspace.h"
#include "src/cpg_solve.h"

  static int i;
#define NSTATES 6
#define NINPUTS 3
#define NHORIZON 21
#define NTOTAL 301
#define NRUNS (NTOTAL - NHORIZON - 1)


  void add_noise(float x[], float var) {
    for (int i = 0; i < NSTATES; ++i) {
      float noise = ((rand() / RAND_MAX) - 0.5) * 2;  // random -1 to 1
      x[i] += noise * var;
    }
  }

  void print_vector(float xn[], int n) {
    for (int i = 0; i < n; ++i) {
      // Serial.println(xn[i]);
      printf("%f, ", xn[i]);
    }
    printf("\n");
  }

  void matrix_vector_mult(int n1,
                          int n2,
                          float matrix[],
                          float vector[],
                          float result_vector[]) {
    // n1 is rows of matrix
    // n2 is cols of matrix, or vector
    int i, j;  // i = row; j = column;
    for (i = 0; i < n1; i++) {
      for (j = 0; j < n2; j++) {
        result_vector[i] += matrix[i * n2 + j] * vector[j];
      }
    }
  }

  void matrix_vector_reset_mult(int n1,
                                int n2,
                                float matrix[],
                                float vector[],
                                float result_vector[]) {
    // n1 is rows of matrix
    // n2 is cols of matrix, or vector
    int i, j;  // i = row; j = column;
    for (i = 0; i < n1; i++) {
      result_vector[i] = 0.0;
      for (j = 0; j < n2; j++) {
        result_vector[i] += matrix[i * n2 + j] * vector[j];
      }
    }
  }

  void system_dynamics(float xn[], float x[], float u[], float A[], float B[], float f[]) {
    matrix_vector_reset_mult(NSTATES, NSTATES, A, x, xn);
    matrix_vector_mult(NSTATES, NINPUTS, B, u, xn);
    for (int i = 0; i < NSTATES; ++i) {
      xn[i] += f[i];
    }
  }

  float compute_norm(float x[], float x_bar[]) {
    float res = 0.0f;
    for (int i = 0; i < NSTATES; ++i) {
      res += (x[i] - x_bar[i]) * (x[i] - x_bar[i]);
    }
    return sqrt(res);
  }

  // May need to save in workspace
  PROGMEM float A[] = { 1.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.05, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.05,
                        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
  PROGMEM float B[] = { 0.000125, 0.0, 0.0,
                        0.0, 0.000125, 0.0,
                        0.0, 0.0, 0.000125,
                        0.005, 0.0, 0.0,
                        0.0, 0.005, 0.0,
                        0.0, 0.0, 0.005 };
  PROGMEM float f[] = { 0.0, 0.0, -0.0122625, 0.0, 0.0, -0.4905 };
  PROGMEM float Q_single = 1e3;
  PROGMEM float xref0[] = { 4, 2, 20, -3, 2, -4.5 };

  float xn[NSTATES] = { 0 };
  float x[NSTATES] = { 4.4, 2.2, 22, -3.3, 2.2, -4.95 };
  float u[NINPUTS] = { 0 };
  float temp = 0;

  void setup() {

    // srand(123);
    Serial.begin(9600);
    // while (!Serial)
    // {
    //     continue;
    // }
    delay(5000);
    Serial.println("Serial initialized");
    Serial.println("Start SCS Rocket Landing");
    Serial.println("============================");

    // UPDATE SOLVER OPTIONS FIRST

    // for ecos
    // cpg_set_solver_abstol(1e-2);
    // cpg_set_solver_reltol(1e-2);
    // cpg_set_solver_maxit(100);

    // for scs
    // cpg_set_solver_eps_abs(1e-2);
    // cpg_set_solver_eps_rel(1e-2);
    // cpg_set_solver_max_iters(100);

    for (int k = 0; k < NRUNS; ++k) {
      //// Update current measurement
      for (int i = 0; i < NSTATES; ++i) {
        cpg_update_param1(i, x[i]);
      }
      printf("x = ");
      print_vector(x, NSTATES);

      // for (int i = 0; i < NHORIZON; ++i)
      // {
      //   printf(" cpg_params_vec[0] = %f\n", cpg_params_vec[i]);
      // }

      //// Update references
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          temp = xref0[j] + (0 - xref0[j]) * (k + i) / (NTOTAL - 1);
          // printf("temp = %f\n", temp);
          cpg_update_param3(i * (NSTATES + NINPUTS) + j, -Q_single * temp);
        }
      }
      // for (int i = 0; i < NHORIZON; ++i)
      // {
      //   printf("  cpg_params_vec[0] = %f\n", cpg_params_vec[i]);
      // }

      // Solve the problem instance
      clock_t start = clock();
      // printf("%d\n", start);
      cpg_solve();
      clock_t end = clock();
      // printf("%d\n", end-start);
      float cpu_time_used = ((float)(end - start)) / CLOCKS_PER_SEC;

      // Get data from the result
      for (i = NSTATES; i < NSTATES + NHORIZON; i++) {
        u[i - NSTATES] = CPG_Result.prim->var2[i];
      }
      printf("u = ");
      print_vector(u, NINPUTS);

      // Simulate the system
      system_dynamics(xn, x, u, A, B, f);
      printf("xn = ");
      print_vector(xn, NSTATES);

      // Update the state
      memcpy(x, xn, NSTATES * (sizeof(float)));
      // print_vector(x, NSTATES);
      add_noise(x, 0.01);
      // print_vector(x, NSTATES);

      // Print objective function value
      // printf("obj = %f\n", CPG_Result.info->obj_val);
      Serial.print("iterations: ");
      Serial.println(CPG_Info.iter);
    }
  }

  void loop() {
  }
}
