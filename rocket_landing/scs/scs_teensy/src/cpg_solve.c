
/*
Auto-generated by CVXPYgen on February 20, 2024 at 23:45:01.
Content: Function definitions.
*/

#include "cpg_solve.h"
#include "cpg_workspace.h"
#include <sys/types.h>
static cpg_int i;
static cpg_int j;

// Update user-defined parameters
void cpg_update_param3(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+0] = val;
  Canon_Outdated.c = 1;
}

void cpg_update_param1(cpg_int idx, cpg_float val){
  cpg_params_vec[idx+186] = val;
  Canon_Outdated.b = 1;
}

// Map user-defined to canonical parameters
void cpg_canonicalize_c(){
  for(i=0; i<392; i++){
    Canon_Params.c[i] = 0;
    for(j=canon_c_map.p[i]; j<canon_c_map.p[i+1]; j++){
      Canon_Params.c[i] += canon_c_map.x[j]*cpg_params_vec[canon_c_map.i[j]];
    }
  }
}

void cpg_canonicalize_b(){
  for(i=0; i<506; i++){
    Canon_Params.b[i] = 0;
    for(j=canon_b_map.p[i]; j<canon_b_map.p[i+1]; j++){
      Canon_Params.b[i] += canon_b_map.x[j]*cpg_params_vec[canon_b_map.i[j]];
    }
  }
}

// Retrieve dual solution in terms of user-defined constraints
void cpg_retrieve_dual(){
  CPG_Dual.d20 = scs_y[306];
  CPG_Dual.d21 = scs_y[307];
  CPG_Dual.d22 = scs_y[308];
  CPG_Dual.d23 = scs_y[309];
  CPG_Dual.d24 = scs_y[310];
  CPG_Dual.d25 = scs_y[311];
  CPG_Dual.d26 = scs_y[312];
  CPG_Dual.d27 = scs_y[313];
  CPG_Dual.d28 = scs_y[314];
  CPG_Dual.d29 = scs_y[315];
  CPG_Dual.d30 = scs_y[316];
  CPG_Dual.d31 = scs_y[317];
  CPG_Dual.d32 = scs_y[318];
  CPG_Dual.d33 = scs_y[319];
  CPG_Dual.d34 = scs_y[320];
  CPG_Dual.d35 = scs_y[321];
  CPG_Dual.d36 = scs_y[322];
  CPG_Dual.d37 = scs_y[323];
  CPG_Dual.d38 = scs_y[324];
  CPG_Dual.d39 = scs_y[325];
}

// Retrieve solver info
void cpg_retrieve_info(){
  CPG_Info.obj_val = (Scs_Info.pobj);
  CPG_Info.iter = Scs_Info.iter;
  CPG_Info.status = Scs_Info.status;
  CPG_Info.pri_res = Scs_Info.res_pri;
  CPG_Info.dua_res = Scs_Info.res_dual;
}

// Solve via canonicalization, canonical solve, retrieval
void cpg_solve(){
  // Canonicalize if necessary
  if (Canon_Outdated.c) {
    cpg_canonicalize_c();
  }
  if (Canon_Outdated.b) {
    cpg_canonicalize_b();
  }
  if (!Scs_Work  ) {
    Scs_Work = scs_init(&Scs_D, &Scs_K, &Canon_Settings);
  } else {
    if ( Canon_Outdated.b&&Canon_Outdated.c) {
      scs_update(Scs_Work, Canon_Params.b, Canon_Params.c);
    } else {
      if ( Canon_Outdated.b) {
        scs_update(Scs_Work, Canon_Params.b, SCS_NULL);
      }
      if ( Canon_Outdated.c) {
        scs_update(Scs_Work, SCS_NULL, Canon_Params.c);
      }
    }
  }
  // Solve with SCS
  scs_solve(Scs_Work, &Scs_Sol, &Scs_Info, (Scs_Work && Canon_Settings.warm_start));
  // Retrieve results
  cpg_retrieve_dual();
  cpg_retrieve_info();
  // Reset flags for outdated canonical parameters
  Canon_Outdated.c = 0;
  Canon_Outdated.b = 0;
}

// Update solver settings
void cpg_set_solver_default_settings(){
  scs_set_default_settings(&Canon_Settings);
}

void cpg_set_solver_normalize(cpg_int normalize_new){
  Canon_Settings.normalize = normalize_new;
}

void cpg_set_solver_scale(cpg_float scale_new){
  Canon_Settings.scale = scale_new;
}

void cpg_set_solver_adaptive_scale(cpg_int adaptive_scale_new){
  Canon_Settings.adaptive_scale = adaptive_scale_new;
}

void cpg_set_solver_rho_x(cpg_float rho_x_new){
  Canon_Settings.rho_x = rho_x_new;
}

void cpg_set_solver_max_iters(cpg_int max_iters_new){
  Canon_Settings.max_iters = max_iters_new;
}

void cpg_set_solver_eps_abs(cpg_float eps_abs_new){
  Canon_Settings.eps_abs = eps_abs_new;
}

void cpg_set_solver_eps_rel(cpg_float eps_rel_new){
  Canon_Settings.eps_rel = eps_rel_new;
}

void cpg_set_solver_eps_infeas(cpg_float eps_infeas_new){
  Canon_Settings.eps_infeas = eps_infeas_new;
}

void cpg_set_solver_alpha(cpg_float alpha_new){
  Canon_Settings.alpha = alpha_new;
}

void cpg_set_solver_time_limit_secs(cpg_float time_limit_secs_new){
  Canon_Settings.time_limit_secs = time_limit_secs_new;
}

void cpg_set_solver_verbose(cpg_int verbose_new){
  Canon_Settings.verbose = verbose_new;
}

void cpg_set_solver_warm_start(cpg_int warm_start_new){
  Canon_Settings.warm_start = warm_start_new;
}

void cpg_set_solver_acceleration_lookback(cpg_int acceleration_lookback_new){
  Canon_Settings.acceleration_lookback = acceleration_lookback_new;
}

void cpg_set_solver_acceleration_interval(cpg_int acceleration_interval_new){
  Canon_Settings.acceleration_interval = acceleration_interval_new;
}

void cpg_set_solver_write_data_filename(const char* write_data_filename_new){
  Canon_Settings.write_data_filename = write_data_filename_new;
}

void cpg_set_solver_log_csv_filename(const char* log_csv_filename_new){
  Canon_Settings.log_csv_filename = log_csv_filename_new;
}

int _open(const char *pathname, int flags, mode_t mode) {
    // Dummy implementation, can be empty
    return -1; // Return an error code to indicate failure
}