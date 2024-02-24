#include "osqp_api_types.h"

#pragma once

#define NSTATES 4

#define NINPUTS 2

#define NHORIZON 20

#define NTOTAL 201

#define SIZE_Q 118

#define SIZE_LU 198

const PROGMEM OSQPFloat mR[4*4] = {
  -100.0,	-0.0,	
  -0.0,	-100.0,	
};

const PROGMEM OSQPFloat A[4*4] = {
  1.0,	0.0,	0.05,	0.0,	
  0.0,	1.0,	0.0,	0.05,	
  0.0,	0.0,	1.0,	0.0,	
  0.0,	0.0,	0.0,	1.0,	
};

const PROGMEM OSQPFloat B[4*2] = {
  0.0012500000000000002,	0.0,	
  0.0,	0.0012500000000000002,	
  0.05,	0.0,	
  0.0,	0.05,	
};

