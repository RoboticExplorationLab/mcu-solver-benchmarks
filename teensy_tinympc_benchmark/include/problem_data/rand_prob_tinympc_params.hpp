#include "types.hpp"

#pragma once

const PROGMEM tinytype Adyn_data[4*4] = {
  -0.40152908392603565,	-0.02830639976257417,	-0.17919088114957826,	-0.29475355550369353,	
  -0.03972674752262319,	0.03759919003958679,	-0.5969966673473691,	0.285755601771632,	
  0.27652610201589994,	0.2638095705791325,	0.2462982143782325,	0.2969796250588069,	
  -0.361870992188192,	-0.2068092672208355,	-0.5257509997742174,	0.19432817804155555,	
};

const PROGMEM tinytype Bdyn_data[4*4] = {
  -0.7722347346617349,	0.35706230766721925,	-0.9743969647757285,	-0.8263764647491993,	
  -0.6698257596794968,	-0.9856274607006166,	-0.01938044604927036,	0.5631942886509629,	
  0.6780850250650643,	-0.9687847755524921,	-0.9446105101426208,	0.24637292395454846,	
  -0.772858718645985,	-0.04025590357872155,	0.3323285775217344,	-0.24169377358216848,	
};

const PROGMEM tinytype Q_data[4] = {1.4066749104503196,2.2105118075544237,3.0189551613299703,0.6647220726581449};

const PROGMEM tinytype Qf_data[4] = {12.660074194052877,19.894606267989815,27.170596451969732,5.982498653923304};

const PROGMEM tinytype R_data[4] = {0.1,0.1,0.1,0.1};

const PROGMEM tinytype umin[4] = {
  -3.0,	
  -3.0,	
  -3.0,	
  -3.0,	
};

const PROGMEM tinytype umax[4] = {
  3.0,	
  3.0,	
  3.0,	
  3.0,	
};

const PROGMEM tinytype xmin[4] = {
  -10000.0,	
  -10000.0,	
  -10000.0,	
  -10000.0,	
};

const PROGMEM tinytype xmax[4] = {
  10000.0,	
  10000.0,	
  10000.0,	
  10000.0,	
};

const PROGMEM tinytype rho_value = 0.1;

const PROGMEM tinytype Kinf_data[4*4] = {
  0.2854819123422703,	0.1032651721438728,	0.5123759872160911,	0.04725460303367805,	
  -0.03758711626364982,	-0.05392795887643256,	0.22112088560272372,	-0.33500784013742424,	
  -0.0029904264037790116,	-0.13129248513377048,	-0.12388471765026493,	0.09760071680872846,	
  0.19253695088863482,	0.07657276917717006,	-0.026160271916732612,	0.012730608391049966,	
};

const PROGMEM tinytype Pinf_data[4*4] = {
  1.53844338529611,	0.014603215547110231,	0.03310888281740098,	-0.009068176228112136,	
  0.014603215547110223,	2.3217690359712826,	0.015303164685569396,	-0.008920253379198814,	
  0.03310888281740076,	0.015303164685569156,	3.1918468061286482,	-0.02425956205517283,	
  -0.009068176228112446,	-0.008920253379199035,	-0.024259562055172355,	0.8237106954058917,	
};

const PROGMEM tinytype Quu_inv_data[4*4] = {
  0.32987572641577756,	-0.2225616544525884,	0.2276564085370578,	-0.4061706535518098,	
  -0.22256165445258796,	1.1244027900827371,	-0.7705594213113027,	1.4756384816935284,	
  0.22765640853705751,	-0.7705594213113026,	0.7646309590638298,	-1.0802864087030872,	
  -0.4061706535518094,	1.4756384816935284,	-1.0802864087030875,	2.4421031025932867,	
};

const PROGMEM tinytype AmBKt_data[4*4] = {
  -0.011454950228310168,	0.00595583043646071,	-0.003729413434020279,	-0.09521812417962117,	
  0.006041129768427567,	0.007946132059564563,	-0.0013433001960819202,	-0.06703139094982583,	
  -0.004801406185386908,	-0.02351884083802469,	0.0025046541177554393,	-0.08600767262344927,	
  -0.03302111980769312,	-0.018063235152838808,	0.02944467684237112,	0.18800466808148406,	
};

const PROGMEM tinytype coeff_d2p_data[4*4] = {
  -8.146955332577477e-14,	9.96876192704832e-14,	2.1733493910797153e-13,	-3.061439990403869e-14,	
  -6.510764150036152e-14,	7.967584925161475e-14,	1.7257723028407668e-13,	-2.492624162631074e-14,	
  -6.855627177060342e-14,	8.348183255790786e-14,	1.8213555663670888e-13,	-2.588294162331195e-14,	
  1.8038522064944829e-13,	-2.152167333235866e-13,	-4.731701142013378e-13,	6.641171987342709e-14,	
};
