
/*
Auto-generated by CVXPYgen on February 16, 2024 at 16:59:26.
Content: Example program for updating parameters, solving, and inspecting the result.
*/

#include <stdio.h>
#include <stdlib.h>
#include "cpg_workspace.h"
#include "cpg_solve.h"
#include "math.h"


static int i;
#define NSTATES 6
#define NINPUTS 3
#define NHORIZON 21
#define NTOTAL 301
#define NRUNS (NTOTAL - NHORIZON - 1)

// void add_noise(double x[])
// {
//   for (int i = 0; i < NSTATES; ++i)
//   {
//     double noise = (double(rand() / RAND_MAX) - 0.5)*2 * 0.01;
//     x[i] += noise;
//   }
// }

void print_vector(double xn[], int n)
{
  for (int i = 0; i < n; ++i)
  {
    // Serial.println(xn[i]);
    printf("%f, ", xn[i]);
  }
  printf("\n");
}

void matrix_vector_mult(int n1,
                        int n2,
                        double matrix[],
                        double vector[],
                        double result_vector[])
{
  // n1 is rows of matrix
  // n2 is cols of matrix, or vector
  int i, j; // i = row; j = column;
  for (i = 0; i < n1; i++)
  {
    for (j = 0; j < n2; j++)
    {
      result_vector[i] += matrix[i * n2 + j] * vector[j];
    }
  }
}

void matrix_vector_reset_mult(int n1,
                              int n2,
                              double matrix[],
                              double vector[],
                              double result_vector[])
{
  // n1 is rows of matrix
  // n2 is cols of matrix, or vector
  int i, j; // i = row; j = column;
  for (i = 0; i < n1; i++)
  {
    result_vector[i] = 0.0;
    for (j = 0; j < n2; j++)
    {
      result_vector[i] += matrix[i * n2 + j] * vector[j];
    }
  }
}

void system_dynamics(double xn[], double x[], double u[], double A[], double B[], double f[])
{
  matrix_vector_reset_mult(NSTATES, NSTATES, A, x, xn);
  matrix_vector_mult(NSTATES, NINPUTS, B, u, xn);
  for (int i = 0; i < NSTATES; ++i)
  {
    xn[i] += f[i];
  }
}

double compute_norm(double x[], double x_bar[])
{
  double res = 0.0f;
  for (int i = 0; i < NSTATES; ++i)
  {
    res += (x[i] - x_bar[i]) * (x[i] - x_bar[i]);
  }
  return sqrt(res);
}

// May need to save in workspace
const double A[] = {1.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0, 0.05, 0.0,
                   0.0, 0.0, 1.0, 0.0, 0.0, 0.05,
                   0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
const double B[] = {0.000125, 0.0, 0.0,
                   0.0, 0.000125, 0.0,
                   0.0, 0.0, 0.000125,
                   0.005, 0.0, 0.0,
                   0.0, 0.005, 0.0,
                   0.0, 0.0, 0.005};
const double f[] = {0.0, 0.0, -0.0122625, 0.0, 0.0, -0.4905};
const double Q_single = 1e3;
const double xref0[] = {4, 2, 20, -3, 2, -4.5};

double xn[NSTATES] = {0};
double x[NSTATES] = {4.4, 2.2, 22, -3.3, 2.2, -4.95};
double u[NINPUTS] = {0};
double temp = 0;

int main(int argc, char *argv[]){
  cpg_set_solver_abstol(1e-2);
  cpg_set_solver_reltol(1e-2);

  for (int k = 0; k < NRUNS; ++k) {
    //// Update current measurement
    for (int i = 0; i < NSTATES; ++i)
    {
      cpg_update_param1(i, x[i]);
    }
    printf(" x = ");
    print_vector(x, NSTATES);

    // for (int i = 0; i < NHORIZON; ++i)
    // {
    //   printf(" cpg_params_vec[0] = %f\n", cpg_params_vec[i]);
    // }

    // Solve the problem instance
    cpg_solve();

    // Get data from the result
    for (i=NSTATES; i<NSTATES+NHORIZON; i++) {
      u[i-NSTATES] = CPG_Result.prim->var2[i];
    }
    printf("u = ");
    print_vector(u, NINPUTS);

    // Simulate the system
    system_dynamics(xn, x, u, A, B, f);
    printf("xn = ");
    print_vector(xn, NSTATES);

    // Update the state
    memcpy(x, xn, NSTATES * (sizeof(double)));

    // Print objective function value
    // printf("obj = %f\n", CPG_Result.info->obj_val);

    //// Print primal solution
    // for(i=0; i<186; i++) {
    //   printf("var2[%d] = %f\n", i, CPG_Result.prim->var2[i]);
    // }
  }
  return 0;
}
