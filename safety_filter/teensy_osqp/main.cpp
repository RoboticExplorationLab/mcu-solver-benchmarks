#include <stdio.h>
#include "osqp.h"
#include "osqp_data_workspace.h"


int main() {
  OSQPInt exitflag;

  printf( "Embedded test program for vector updates.\n");

  exitflag = osqp_solve( &osqp_data_solver );

  if( exitflag > 0 ) {
    printf( "  OSQP errored: %s\n", osqp_error_message(exitflag));
    return (int)exitflag;
  } else {
    printf( "  Solved workspace with no error.\n" );
  }
  for (int i = 0; i < 267; i++) {
    printf("  Solution: %f\n", osqp_data_solver.solution->x[i]);
  }
  // printf("  Solution: %f\n", osqp_data_solver.solution->x[0]);
}
