/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#include <stdio.h>
#include <iostream>
#include "osqp.h"
#include "osqp_data_workspace.h"
#include "rand_prob_osqp_xbar.h"
#include "math.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

void add_noise(OSQPFloat x[])
{
  for (int i = 0; i < NSTATES; ++i)
  {
    OSQPFloat noise = (float(rand() / RAND_MAX) - 0.5)*2 * 0.01;
    x[i] += noise;
  }
}

void print_vector(OSQPFloat xn[], int n)
{
  for (int i = 0; i < n; ++i)
  {
    Serial.println(xn[i]);
  }
}

void matrix_vector_mult(int n1,
                        int n2,
                        OSQPFloat matrix[],
                        OSQPFloat vector[],
                        OSQPFloat result_vector[])
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
                              OSQPFloat matrix[],
                              OSQPFloat vector[],
                              OSQPFloat result_vector[])
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

void system_dynamics(OSQPFloat xn[], OSQPFloat x[], OSQPFloat u[], OSQPFloat A[], OSQPFloat B[])
{
  matrix_vector_reset_mult(NSTATES, NSTATES, A, x, xn);
  matrix_vector_mult(NSTATES, NINPUTS, B, u, xn);
}

void compute_q(OSQPFloat q[], OSQPFloat Q_data[], OSQPFloat Qf_data[], OSQPFloat Xref[])
// Xref is Nh x Nx, pointer at current step
// q is Nh x (Nx + 1) + Nh x Nu
// Q_data is Nx x Nx
{
  for (int i = 0; i < NHORIZON; ++i)
  {
    matrix_vector_reset_mult(NSTATES, NSTATES, Q_data, Xref + (i * NSTATES), q + (i * NSTATES));
  }
  matrix_vector_reset_mult(NSTATES, NSTATES, Qf_data, Xref + ((NHORIZON)*NSTATES), q + ((NHORIZON)*NSTATES));
}

void compute_bound(OSQPFloat bnew[], OSQPFloat xn[], OSQPFloat xb, OSQPFloat ub, int size)
{
  for (int i = 0; i < NSTATES; ++i)
  {
    bnew[i] = -xn[i]; // only the first is current state
  }
  for (int i = (NHORIZON + 1) * NSTATES; i < (NHORIZON + 1) * NSTATES * 2; ++i)
  {
    bnew[i] = xb; // bounds on x
  }
  for (int i = (NHORIZON + 1) * NSTATES * 2; i < size; ++i)
  {
    bnew[i] = ub; // bounds on u
  }
}

OSQPFloat compute_norm(OSQPFloat x[], OSQPFloat x_bar[])
{
  OSQPFloat res = 0.0f;
  for (int i = 0; i < NSTATES; ++i)
  {
    res += (x[i] - x_bar[i]) * (x[i] - x_bar[i]);
  }
  return sqrt(res);
}

OSQPInt exitflag;
OSQPFloat xn[NSTATES] = {0};
OSQPFloat x[NSTATES] = {0};
OSQPFloat q_new[SIZE_Q] = {0};
OSQPFloat l_new[SIZE_LU] = {0};
OSQPFloat u_new[SIZE_LU] = {0};
OSQPFloat xmin = -10000;
OSQPFloat xmax = 10000;
OSQPFloat umin = -3;
OSQPFloat umax = 3;

void setup()
// int main()
{
  srand(123);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // start serial terminal
  Serial.begin(9600);
  while (!Serial)
  { // wait to connect
    continue;
  }

  Serial.println("Start");
  for (int step = 0; step < NTOTAL - NHORIZON; step++)
  // for (int step = 0; step < 5; step++)
  {
    compute_q(q_new, mQ, mQf, &(Xref_data[(step) * NSTATES]));
    compute_bound(l_new, x, xmin, umin, SIZE_LU);
    compute_bound(u_new, x, xmax, umax, SIZE_LU);
    osqp_update_data_vec(&osqp_data_solver, q_new, l_new, u_new);
    unsigned long start = micros();
    exitflag = osqp_solve(&osqp_data_solver);
    unsigned long end = micros();
    OSQPFloat norm = compute_norm(x, &(Xref_data[step * NSTATES]));

    system_dynamics(xn, x, (osqp_data_solver.solution->x) + (NHORIZON + 1) * NSTATES, A, B);
    // printf("control\n");
    // print_vector((osqp_data_solver.solution->x) + (NHORIZON + 1) * NSTATES, NINPUTS);
    add_noise(xn);
    memcpy(x, xn, NSTATES * (sizeof(OSQPFloat)));
    printf("%10.4f %10d %10d\n", norm, osqp_data_solver.info->iter, end - start);
  }
}

void loop()
{
}