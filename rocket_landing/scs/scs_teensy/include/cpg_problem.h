#pragma once

#define NSTATES 6

#define NINPUTS 3

#define NHORIZON 32

#define NTOTAL 301

#define Q_single 1000.0

const PROGMEM float A[6*6] = {
  1.0,	0.0,	0.0,	0.05,	0.0,	0.0,	
  0.0,	1.0,	0.0,	0.0,	0.05,	0.0,	
  0.0,	0.0,	1.0,	0.0,	0.0,	0.05,	
  0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	
  0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	
  0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	
};

const PROGMEM float B[6*3] = {
  0.000125,	0.0,	0.0,	
  0.0,	0.000125,	0.0,	
  0.0,	0.0,	0.000125,	
  0.005,	0.0,	0.0,	
  0.0,	0.005,	0.0,	
  0.0,	0.0,	0.005,	
};

const PROGMEM float f[6] = {
  0.0,	
  0.0,	
  -0.0122625,	
  0.0,	
  0.0,	
  -0.4905,	
};

