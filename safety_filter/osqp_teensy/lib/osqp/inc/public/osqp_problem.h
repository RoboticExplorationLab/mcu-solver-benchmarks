#include "osqp_api_types.h"

#pragma once

#define NSTATES 6

#define NINPUTS 3

#define NHORIZON 30

#define NTOTAL 201

#define SIZE_Q 267

#define SIZE_LU 447

const PROGMEM OSQPFloat mR[6*6] = {
  -100.0,	-0.0,	-0.0,	
  -0.0,	-100.0,	-0.0,	
  -0.0,	-0.0,	-100.0,	
};

const PROGMEM OSQPFloat A[6*6] = {
  1.0,	0.0,	0.0,	0.05,	0.0,	0.0,	
  0.0,	1.0,	0.0,	0.0,	0.05,	0.0,	
  0.0,	0.0,	1.0,	0.0,	0.0,	0.05,	
  0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	
  0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	
  0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	
};

const PROGMEM OSQPFloat B[6*3] = {
  0.0012500000000000002,	0.0,	0.0,	
  0.0,	0.0012500000000000002,	0.0,	
  0.0,	0.0,	0.0012500000000000002,	
  0.05,	0.0,	0.0,	
  0.0,	0.05,	0.0,	
  0.0,	0.0,	0.05,	
};

