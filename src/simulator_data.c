#include "simulator_data.h"
/* AUTO-GENERATED FROM simulator_almanac_generator.py and data/week763.alm*/

u16 simulation_week_number = 1787;

double simulation_sats_pos[31][3];

double simulation_sats_vel[31][3];

u32 simulation_fake_carrier_bias[31];

u8 simulation_num_almanacs = 31;

const almanac_t simulation_almanacs[31] = {
{ 
  .kepler = { 
    .ecc       = 0.002888,
    .inc       = 0.960802,
    .omegadot  = -0.000000,
    .sqrta     = 5153.633301,
    .omega0    = -0.818239,
    .w         = 0.321128,
    .m0        = 1.872279,
    .af0       = 0.000008,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 1
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.013428,
    .inc       = 0.939314,
    .omegadot  = -0.000000,
    .sqrta     = 5153.531738,
    .omega0    = -0.846764,
    .w         = -2.446993,
    .m0        = 2.488397,
    .af0       = 0.000488,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 2
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.016847,
    .inc       = 0.937600,
    .omegadot  = -0.000000,
    .sqrta     = 5153.723633,
    .omega0    = -2.021208,
    .w         = 1.373169,
    .m0        = 2.664127,
    .af0       = 0.000349,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 3
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.010565,
    .inc       = 0.938349,
    .omegadot  = -0.000000,
    .sqrta     = 5153.663574,
    .omega0    = -0.830694,
    .w         = 1.086847,
    .m0        = -0.403849,
    .af0       = 0.000007,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 4
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.003571,
    .inc       = 0.947841,
    .omegadot  = -0.000000,
    .sqrta     = 5153.510254,
    .omega0    = 0.220327,
    .w         = 0.330886,
    .m0        = -1.481788,
    .af0       = -0.000380,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 5
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.007160,
    .inc       = 0.973727,
    .omegadot  = -0.000000,
    .sqrta     = 5153.596191,
    .omega0    = 2.342325,
    .w         = -2.840829,
    .m0        = 1.241122,
    .af0       = 0.000333,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 7
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.013593,
    .inc       = 0.996982,
    .omegadot  = -0.000000,
    .sqrta     = 5153.445801,
    .omega0    = 2.440640,
    .w         = -2.782502,
    .m0        = 0.687856,
    .af0       = 0.000012,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 8
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.017078,
    .inc       = 0.982259,
    .omegadot  = -0.000000,
    .sqrta     = 5153.644043,
    .omega0    = 2.321551,
    .w         = 1.786317,
    .m0        = 2.326993,
    .af0       = 0.000319,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 9
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.013494,
    .inc       = 0.942640,
    .omegadot  = -0.000000,
    .sqrta     = 5153.662109,
    .omega0    = 0.237867,
    .w         = 0.861145,
    .m0        = -1.187092,
    .af0       = -0.000121,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 10
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.014848,
    .inc       = 0.890131,
    .omegadot  = -0.000000,
    .sqrta     = 5153.634277,
    .omega0    = -1.138635,
    .w         = 1.274041,
    .m0        = 1.431038,
    .af0       = -0.000479,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 11
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.005056,
    .inc       = 0.987389,
    .omegadot  = -0.000000,
    .sqrta     = 5153.601562,
    .omega0    = -2.878973,
    .w         = 0.384712,
    .m0        = 0.843477,
    .af0       = 0.000197,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 12
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.005360,
    .inc       = 0.977837,
    .omegadot  = -0.000000,
    .sqrta     = 5153.559082,
    .omega0    = 1.377231,
    .w         = 2.220456,
    .m0        = -2.691997,
    .af0       = 0.000011,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 13
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.007339,
    .inc       = 0.970431,
    .omegadot  = -0.000000,
    .sqrta     = 5153.591797,
    .omega0    = 1.345702,
    .w         = -1.999234,
    .m0        = -2.357275,
    .af0       = 0.000183,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 14
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.006239,
    .inc       = 0.937067,
    .omegadot  = -0.000000,
    .sqrta     = 5153.610840,
    .omega0    = 1.235251,
    .w         = 0.273510,
    .m0        = -2.960076,
    .af0       = -0.000173,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 15
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.007709,
    .inc       = 0.988000,
    .omegadot  = -0.000000,
    .sqrta     = 5153.521484,
    .omega0    = -2.860449,
    .w         = 0.174079,
    .m0        = -1.328606,
    .af0       = -0.000224,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 16
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.009278,
    .inc       = 0.967908,
    .omegadot  = -0.000000,
    .sqrta     = 5153.522461,
    .omega0    = -1.827968,
    .w         = -2.104891,
    .m0        = -2.298537,
    .af0       = -0.000083,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 17
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.015035,
    .inc       = 0.926407,
    .omegadot  = -0.000000,
    .sqrta     = 5153.620117,
    .omega0    = 0.216093,
    .w         = -2.049409,
    .m0        = -0.665323,
    .af0       = 0.000312,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 18
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.010157,
    .inc       = 0.965026,
    .omegadot  = -0.000000,
    .sqrta     = 5153.547852,
    .omega0    = -1.776980,
    .w         = 0.411913,
    .m0        = -3.058934,
    .af0       = -0.000450,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 19
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.005853,
    .inc       = 0.927390,
    .omegadot  = -0.000000,
    .sqrta     = 5153.476562,
    .omega0    = 0.162982,
    .w         = 1.314663,
    .m0        = -0.106709,
    .af0       = 0.000177,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 20
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.021363,
    .inc       = 0.932255,
    .omegadot  = -0.000000,
    .sqrta     = 5153.721191,
    .omega0    = -0.829072,
    .w         = -2.041057,
    .m0        = 0.327476,
    .af0       = -0.000354,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 21
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.007021,
    .inc       = 0.924178,
    .omegadot  = -0.000000,
    .sqrta     = 5153.664551,
    .omega0    = 0.218072,
    .w         = -2.064006,
    .m0        = -1.216956,
    .af0       = 0.000225,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 22
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.009099,
    .inc       = 0.952095,
    .omegadot  = -0.000000,
    .sqrta     = 5153.513184,
    .omega0    = 1.281210,
    .w         = -2.798111,
    .m0        = 2.897357,
    .af0       = -0.000010,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 23
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.002036,
    .inc       = 0.957081,
    .omegadot  = -0.000000,
    .sqrta     = 5153.693359,
    .omega0    = 2.308488,
    .w         = 0.311778,
    .m0        = 2.115817,
    .af0       = -0.000027,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 24
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.003447,
    .inc       = 0.976124,
    .omegadot  = -0.000000,
    .sqrta     = 5153.581055,
    .omega0    = -2.920817,
    .w         = 0.632882,
    .m0        = 0.064184,
    .af0       = 0.000021,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 25
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.021486,
    .inc       = 0.975243,
    .omegadot  = -0.000000,
    .sqrta     = 5153.640137,
    .omega0    = 1.368661,
    .w         = 1.275835,
    .m0        = 2.792296,
    .af0       = 0.000114,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 26
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.001075,
    .inc       = 0.962641,
    .omegadot  = -0.000000,
    .sqrta     = 5153.621094,
    .omega0    = -1.873221,
    .w         = 0.052962,
    .m0        = -2.123342,
    .af0       = -0.000020,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 27
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.019157,
    .inc       = 0.986508,
    .omegadot  = -0.000000,
    .sqrta     = 5153.622070,
    .omega0    = -2.854617,
    .w         = -1.716683,
    .m0        = -1.431703,
    .af0       = 0.000346,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 28
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.001677,
    .inc       = 0.968627,
    .omegadot  = -0.000000,
    .sqrta     = 5153.676758,
    .omega0    = -1.818947,
    .w         = -0.929559,
    .m0        = 0.550541,
    .af0       = 0.000525,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 29
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.000489,
    .inc       = 0.959286,
    .omegadot  = -0.000000,
    .sqrta     = 5153.666504,
    .omega0    = 2.397483,
    .w         = 2.656720,
    .m0        = 1.584534,
    .af0       = 0.000001,
    .af1       = -0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 30
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 0,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.008084,
    .inc       = 0.978676,
    .omegadot  = -0.000000,
    .sqrta     = 5153.762695,
    .omega0    = 2.348260,
    .w         = -0.630742,
    .m0        = 1.101170,
    .af0       = 0.000334,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 31
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .kepler = { 
    .ecc       = 0.011426,
    .inc       = 0.948308,
    .omegadot  = -0.000000,
    .sqrta     = 5153.707520,
    .omega0    = 0.309108,
    .w         = -0.173257,
    .m0        = 1.856852,
    .af0       = -0.000453,
    .af1       = 0.000000,
  },
  .sid = { 
    .code = 0,
    .sat = 32
  },
  .toa = {
    .wn = 1787,
    .tow = 233472.000000
  },
  .ura = 900.000000,
  .fit_interval = 518400,
  .healthy   = 1,
  .valid     = 1,
},
};
