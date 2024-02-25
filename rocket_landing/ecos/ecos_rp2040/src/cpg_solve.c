
/*
Auto-generated by CVXPYgen on February 05, 2024 at 13:52:40.
Content: Function definitions.
*/

#include "cpg_solve.h"
#include "cpg_workspace.h"

static cpg_int i;
static cpg_int j;

// Update user-defined parameters
void cpg_update_d(cpg_float val){
  cpg_params_vec[0] = val;
  Canon_Outdated.h = 1;
}

void cpg_update_A(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+1] = val;
  Canon_Outdated.G = 1;
}

void cpg_update_b(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+51] = val;
  Canon_Outdated.h = 1;
}

void cpg_update_F(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+56] = val;
  Canon_Outdated.A = 1;
}

void cpg_update_g(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+106] = val;
  Canon_Outdated.b = 1;
}

// Map user-defined to canonical parameters
void cpg_canonicalize_A(){
  for(i=0; i<50; i++){
    Canon_Params.A->x[i] = 0;
    for(j=canon_A_map.p[i]; j<canon_A_map.p[i+1]; j++){
      Canon_Params.A->x[i] += canon_A_map.x[j]*cpg_params_vec[canon_A_map.i[j]];
    }
  }
}

void cpg_canonicalize_b(){
  for(i=0; i<5; i++){
    Canon_Params.b[i] = 0;
    for(j=canon_b_map.p[i]; j<canon_b_map.p[i+1]; j++){
      Canon_Params.b[i] += canon_b_map.x[j]*cpg_params_vec[canon_b_map.i[j]];
    }
  }
}

void cpg_canonicalize_G(){
  for(i=0; i<50; i++){
    Canon_Params.G->x[i] = 0;
    for(j=canon_G_map.p[i]; j<canon_G_map.p[i+1]; j++){
      Canon_Params.G->x[i] += canon_G_map.x[j]*cpg_params_vec[canon_G_map.i[j]];
    }
  }
}

void cpg_canonicalize_h(){
  for(i=0; i<6; i++){
    Canon_Params.h[i] = 0;
    for(j=canon_h_map.p[i]; j<canon_h_map.p[i+1]; j++){
      Canon_Params.h[i] += canon_h_map.x[j]*cpg_params_vec[canon_h_map.i[j]];
    }
  }
}

// Retrieve primal solution in terms of user-defined variables
void cpg_retrieve_prim(){
  CPG_Prim.x[0] = ecos_workspace->x[0];
  CPG_Prim.x[1] = ecos_workspace->x[1];
  CPG_Prim.x[2] = ecos_workspace->x[2];
  CPG_Prim.x[3] = ecos_workspace->x[3];
  CPG_Prim.x[4] = ecos_workspace->x[4];
  CPG_Prim.x[5] = ecos_workspace->x[5];
  CPG_Prim.x[6] = ecos_workspace->x[6];
  CPG_Prim.x[7] = ecos_workspace->x[7];
  CPG_Prim.x[8] = ecos_workspace->x[8];
  CPG_Prim.x[9] = ecos_workspace->x[9];
}

// Retrieve dual solution in terms of user-defined constraints
void cpg_retrieve_dual(){
  CPG_Dual.d0 = ecos_workspace->y[0];
  CPG_Dual.d1[0] = ecos_workspace->z[0];
  CPG_Dual.d1[1] = ecos_workspace->z[1];
  CPG_Dual.d1[2] = ecos_workspace->z[2];
  CPG_Dual.d1[3] = ecos_workspace->z[3];
  CPG_Dual.d1[4] = ecos_workspace->z[4];
}

// Retrieve solver info
void cpg_retrieve_info(){
  CPG_Info.obj_val = (ecos_workspace->info->pcost);
  CPG_Info.iter = ecos_workspace->info->iter;
  CPG_Info.status = ecos_flag;
  CPG_Info.pri_res = ecos_workspace->info->pres;
  CPG_Info.dua_res = ecos_workspace->info->dres;
}

// Solve via canonicalization, canonical solve, retrieval
void cpg_solve(){
  // Canonicalize if necessary
  if (Canon_Outdated.A) {
    cpg_canonicalize_A();
  }
  for (i=0; i<50; i++){
    Canon_Params_conditioning.A->x[i] = Canon_Params.A->x[i];
  }
  if (Canon_Outdated.b) {
    cpg_canonicalize_b();
  }
  for (i=0; i<5; i++){
    Canon_Params_conditioning.b[i] = Canon_Params.b[i];
  }
  if (Canon_Outdated.G) {
    cpg_canonicalize_G();
  }
  for (i=0; i<50; i++){
    Canon_Params_conditioning.G->x[i] = Canon_Params.G->x[i];
  }
  if (Canon_Outdated.h) {
    cpg_canonicalize_h();
  }
  for (i=0; i<6; i++){
    Canon_Params_conditioning.h[i] = Canon_Params.h[i];
  }
  if (!ecos_workspace  ) {
    ecos_workspace = ECOS_setup(10, 6, 5, 0, 1, (int *) &ecos_q, 0, Canon_Params_conditioning.G->x, Canon_Params_conditioning.G->p, Canon_Params_conditioning.G->i, Canon_Params_conditioning.A->x, Canon_Params_conditioning.A->p, Canon_Params_conditioning.A->i, Canon_Params_conditioning.c, Canon_Params_conditioning.h, Canon_Params_conditioning.b);
  } else {
    if ( Canon_Outdated.A||Canon_Outdated.b||Canon_Outdated.G) {
      ECOS_updateData(ecos_workspace, Canon_Params_conditioning.G->x, Canon_Params_conditioning.A->x, Canon_Params_conditioning.c, Canon_Params_conditioning.h, Canon_Params_conditioning.b);
    } else {
      if ( Canon_Outdated.h) {
        for (i=0; i<6; i++) { ecos_updateDataEntry_h(ecos_workspace, i, Canon_Params_conditioning.h[i]); };
      }
    }
  }
  ecos_workspace->stgs->feastol = Canon_Settings.feastol;
  ecos_workspace->stgs->abstol = Canon_Settings.abstol;
  ecos_workspace->stgs->reltol = Canon_Settings.reltol;
  ecos_workspace->stgs->feastol_inacc = Canon_Settings.feastol_inacc;
  ecos_workspace->stgs->abstol_inacc = Canon_Settings.abstol_inacc;
  ecos_workspace->stgs->reltol_inacc = Canon_Settings.reltol_inacc;
  ecos_workspace->stgs->maxit = Canon_Settings.maxit;
  // Solve with ECOS
  ecos_flag = ECOS_solve(ecos_workspace);
  // Retrieve results
  cpg_retrieve_prim();
  cpg_retrieve_dual();
  cpg_retrieve_info();
  // Reset flags for outdated canonical parameters
  Canon_Outdated.A = 0;
  Canon_Outdated.b = 0;
  Canon_Outdated.G = 0;
  Canon_Outdated.h = 0;
}

// Update solver settings
void cpg_set_solver_default_settings(){
  Canon_Settings.feastol = 1e-8;
  Canon_Settings.abstol = 1e-8;
  Canon_Settings.reltol = 1e-8;
  Canon_Settings.feastol_inacc = 1e-4;
  Canon_Settings.abstol_inacc = 5e-5;
  Canon_Settings.reltol_inacc = 5e-5;
  Canon_Settings.maxit = 100;
}

void cpg_set_solver_feastol(cpg_float feastol_new){
  Canon_Settings.feastol = feastol_new;
}

void cpg_set_solver_abstol(cpg_float abstol_new){
  Canon_Settings.abstol = abstol_new;
}

void cpg_set_solver_reltol(cpg_float reltol_new){
  Canon_Settings.reltol = reltol_new;
}

void cpg_set_solver_feastol_inacc(cpg_float feastol_inacc_new){
  Canon_Settings.feastol_inacc = feastol_inacc_new;
}

void cpg_set_solver_abstol_inacc(cpg_float abstol_inacc_new){
  Canon_Settings.abstol_inacc = abstol_inacc_new;
}

void cpg_set_solver_reltol_inacc(cpg_float reltol_inacc_new){
  Canon_Settings.reltol_inacc = reltol_inacc_new;
}

void cpg_set_solver_maxit(cpg_int maxit_new){
  Canon_Settings.maxit = maxit_new;
}