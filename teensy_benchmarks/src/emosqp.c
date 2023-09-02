/*
 * This file was autogenerated by OSQP on Fri Sep  1 21:32:41 2023
 * 
 * This file contains a sample solver to run the embedded code.
 */

#include <stdio.h>
#include "osqp.h"
#include "mpc_mat_workspace.h"

int main_old() {
  OSQPInt exitflag;

  printf( "Embedded test program for vector updates.\n");

  exitflag = osqp_solve( &mpc_mat_solver );

  if( exitflag > 0 ) {
    printf( "  OSQP errored: %s\n", osqp_error_message(exitflag));
    return (int)exitflag;
  } else {
    printf( "  Solved workspace with no error.\n" );
  }
}
