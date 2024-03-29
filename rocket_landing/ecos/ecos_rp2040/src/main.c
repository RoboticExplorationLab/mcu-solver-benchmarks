
/*
Auto-generated by CVXPYgen on February 05, 2024 at 13:52:40.
Content: Example program for updating parameters, solving, and inspecting the result.
*/

#include <stdio.h>
#include "cpg_workspace.h"
#include "cpg_solve.h"

static int i;

int main(int argc, char *argv[]){

  // Update first entry of every user-defined parameter
  cpg_update_d(13.59415410702322546399);
  cpg_update_A(0, -0.87810789324034199677);
  cpg_update_b(0, -0.65325026779203565486);
  cpg_update_F(0, 1.01985472936923726728);
  cpg_update_g(0, 2.06046216253297664878);

  // Solve the problem instance
  cpg_solve();

  // Print objective function value
  printf("obj = %f\n", CPG_Result.info->obj_val);

  for(i = 0; i< 300; i++){
      printf("i = %i , obj = %f\n", i, CPG_Result.info->obj_val);}
  // Print primal solution
  for(i=0; i<10; i++) {
    printf("x[%d] = %f\n", i, CPG_Result.prim->x[i]);
  }

  // Print dual solution
  printf("d0 = %f\n", CPG_Result.dual->d0);
  for(i=0; i<5; i++) {
    printf("d1[%d] = %f\n", i, CPG_Result.dual->d1[i]);
  }

  return 0;

}
