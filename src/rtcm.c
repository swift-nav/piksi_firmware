/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * Based on RTKLIB, Copyright (C) 2007-2013 by T.TAKASU, All rights reserved.
 * Released under BSD license.
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm.h"

#include <math.h>
#include <libswiftnav/edc.h>

#define RTCM3PREAMB 0xD3  /* rtcm ver.3 frame preamble */

#define ROUND(x)    ((s32)floor((x) + 0.5))
#define ROUND_U(x)  ((u32)floor((x) + 0.5))

#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */
#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */

#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */



#define P2_10       0.0009765625          /* 2^-10 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */

/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : unsigned char *buff IO byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
*         (unsigned) int I      unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
void setbitu(u8 *buff, u32 pos, u32 len, u32 data)
{
  u32 mask = 1u << (len - 1);

  if (len <= 0 || 32 < len) return;

  for (u32 i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask)
      buff[i / 8] |= 1u << (7 - i % 8);
    else
      buff[i / 8] &= ~(1u << (7 - i % 8));
  }
}
void setbits(u8 *buff, u32 pos, u32 len, s32 data)
{
  if (data < 0)
    data |= 1 << (len - 1);
  else
    data &= ~(1 << (len - 1));   /* set sign bit */
  setbitu(buff, pos, len, (u32)data);
}
/*
static double myfmod(double x, double y)
{
  return x - (long long int)(x/y) * y;
}
*/
/* carrier-phase - pseudorange in cycle --------------------------------------*/
static double cp_pr(double cp, double pr_cyc)
{
    double x = fmod(cp-pr_cyc+1500.0,3000.0);
    if (x < 0)
      x += 3000;
    x -= 1500.0;
    return x;
}


/* generate obs field data gps -----------------------------------------------*/
static void gen_obs_gps(rtcm_t *rtcm, const navigation_measurement_t *data,
    int *code1, int *pr1, int *ppr1, int *lock1, int *amb, int *cnr1)
{
  double lam1, pr1c = 0.0, ppr;
  /*int lt1, lt2;*/
  (void)rtcm;

  lam1 = CLIGHT / FREQ1;
  *pr1 = *amb = 0;
  if (ppr1) *ppr1 = 0xFFF80000;   /* invalid values */

  if (data->raw_pseudorange != 0.0) {
    /* L1 peudorange */
    *amb = (int)floor(data->raw_pseudorange / PRUNIT_GPS);
    *pr1 = ROUND((data->raw_pseudorange - *amb * PRUNIT_GPS) / 0.02);
    pr1c = *pr1 * 0.02 + *amb * PRUNIT_GPS;

    /* L1 phaserange - L1 pseudorange */
    ppr = cp_pr(data->carrier_phase, pr1c / lam1);
    /*printf("%02d - cp: %g, pr1c: %g, ppr: %g\n", data->prn+1, data->carrier_phase, pr1c/lam1, ppr);*/
    if (ppr1) *ppr1 = ROUND(ppr * lam1 / 0.0005);
  }
  /*lt1 = locktime(data->time, rtcm->lltime[data->sat - 1], data->LLI[0]);*/

  /* TODO: implement lock time info. For now say we have been locked for ages. */
  if (lock1) *lock1 = 127;//to_lock(lt1);
  if (cnr1 ) *cnr1 = (u8)((10.0*log10(data->snr) + 40.0) * 4.0);
  if (code1) *code1 = 0;
}


/* encode rtcm header --------------------------------------------------------*/
static int encode_head(int type, rtcm_t *rtcm, int sync, int nsat)
{
  int i = 24, epoch;

  setbitu(rtcm->buff, i, 12, type); i += 12;        /* message no */
  setbitu(rtcm->buff, i, 12, 0);    i += 12;        /* ref station id */

  epoch = ROUND(rtcm->time.tow / 0.001);
  setbitu(rtcm->buff, i, 30, epoch); i += 30;       /* gps epoch time */

  setbitu(rtcm->buff, i, 1, sync); i += 1;          /* synchronous gnss flag */
  setbitu(rtcm->buff, i, 5, nsat); i += 5;          /* no of satellites */
  setbitu(rtcm->buff, i, 1, 0);    i += 1;          /* smoothing indicator */
  setbitu(rtcm->buff, i, 3, 0);    i += 3;          /* smoothing interval */
  return i;
}



/* encode type 1002: extended L1-only gps rtk observables --------------------*/
static int encode_type1002(rtcm_t *rtcm, int sync)
{
  int i, j;
  int code1, pr1, ppr1, lock1, amb, cnr1;

  /* encode header */
  i = encode_head(1002, rtcm, sync, rtcm->n);

  for (j = 0; j < rtcm->n; j++) {

    /* generate obs field data gps */
    gen_obs_gps(rtcm, &(rtcm->obs[j]), &code1, &pr1, &ppr1, &lock1, &amb,
                &cnr1);

    setbitu(rtcm->buff, i, 6, rtcm->obs[j].prn+1  ); i += 6;
    setbitu(rtcm->buff, i, 1, code1); i += 1;
    setbitu(rtcm->buff, i, 24, pr1  ); i += 24;
    setbits(rtcm->buff, i, 20, ppr1 ); i += 20;
    setbitu(rtcm->buff, i, 7, lock1); i += 7;
    setbitu(rtcm->buff, i, 8, amb  ); i += 8;
    setbitu(rtcm->buff, i, 8, cnr1 ); i += 8;
  }
  rtcm->nbit = i;
  return 1;
}




/* encode type 1019: gps ephemerides -----------------------------------------*/
static int encode_type1019(rtcm_t *rtcm, int sync)
{
  ephemeris_t *swift_eph;
  unsigned int sqrtA, e;
  int i = 24, week, toe, toc, i0, OMG0, omg, M0, deln, idot, OMGd, crs,
      crc;
  int cus, cuc, cis, cic, af0, af1, af2, tgd;

  (void)sync;

  swift_eph = rtcm->eph;

  week = swift_eph->toe.wn % 1024;
  toe  = ROUND(swift_eph->toe.tow / 16.0);
  toc  = ROUND(swift_eph->toc.tow / 16.0);
  sqrtA = ROUND_U(swift_eph->sqrta / P2_19);
  e    = ROUND_U(swift_eph->ecc / P2_33);
  i0   = ROUND(swift_eph->inc / P2_31 / SC2RAD);
  OMG0 = ROUND(swift_eph->omega0 / P2_31 / SC2RAD);
  omg  = ROUND(swift_eph->w / P2_31 / SC2RAD);
  M0   = ROUND(swift_eph->m0 / P2_31 / SC2RAD);
  deln = ROUND(swift_eph->dn / P2_43 / SC2RAD);
  idot = ROUND(swift_eph->inc_dot / P2_43 / SC2RAD);
  OMGd = ROUND(swift_eph->omegadot / P2_43 / SC2RAD);
  crs  = ROUND(swift_eph->crs / P2_5 );
  crc  = ROUND(swift_eph->crc / P2_5 );
  cus  = ROUND(swift_eph->cus / P2_29);
  cuc  = ROUND(swift_eph->cuc / P2_29);
  cis  = ROUND(swift_eph->cis / P2_29);
  cic  = ROUND(swift_eph->cic / P2_29);
  af0  = ROUND(swift_eph->af0 / P2_31);
  af1  = ROUND(swift_eph->af1 / P2_43);
  af2  = ROUND(swift_eph->af2 / P2_55);
  tgd  = ROUND(swift_eph->tgd / P2_31);

  /* TODO: Lots of fields missing from ephemeris!! Just hacked in reasonable
    values here. */
  setbitu(rtcm->buff, i, 12, 1019);  i += 12;
  setbitu(rtcm->buff, i, 6, rtcm->prn+1);    i += 6;
  setbitu(rtcm->buff, i, 10, week);  i += 10;
  setbitu(rtcm->buff, i, 4, 1);      i += 4;
  setbitu(rtcm->buff, i, 2, 0);      i += 2;
  setbits(rtcm->buff, i, 14, idot);  i += 14;
  setbitu(rtcm->buff, i, 8, 1);      i += 8;
  setbitu(rtcm->buff, i, 16, toc);   i += 16;
  setbits(rtcm->buff, i, 8, af2);    i += 8;
  setbits(rtcm->buff, i, 16, af1);   i += 16;
  setbits(rtcm->buff, i, 22, af0);   i += 22;
  setbitu(rtcm->buff, i, 10, 1);     i += 10;
  setbits(rtcm->buff, i, 16, crs);   i += 16;
  setbits(rtcm->buff, i, 16, deln);  i += 16;
  setbits(rtcm->buff, i, 32, M0);    i += 32;
  setbits(rtcm->buff, i, 16, cuc);   i += 16;
  setbitu(rtcm->buff, i, 32, e);     i += 32;
  setbits(rtcm->buff, i, 16, cus);   i += 16;
  setbitu(rtcm->buff, i, 32, sqrtA); i += 32;
  setbitu(rtcm->buff, i, 16, toe);   i += 16;
  setbits(rtcm->buff, i, 16, cic);   i += 16;
  setbits(rtcm->buff, i, 32, OMG0);  i += 32;
  setbits(rtcm->buff, i, 16, cis);   i += 16;
  setbits(rtcm->buff, i, 32, i0);    i += 32;
  setbits(rtcm->buff, i, 16, crc);   i += 16;
  setbits(rtcm->buff, i, 32, omg);   i += 32;
  setbits(rtcm->buff, i, 24, OMGd);  i += 24;
  setbits(rtcm->buff, i, 8, tgd);    i += 8;
  setbitu(rtcm->buff, i, 6, (swift_eph->healthy) ? 0 : 1 ); i += 6;
  setbitu(rtcm->buff, i, 1, 0);      i += 1;
  setbitu(rtcm->buff, i, 1, 1);      i += 1;
  rtcm->nbit = i;
  return 1;
}


#if 0
/* encode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
static int encode_msm7(rtcm_t *rtcm, int sys, int sync)
{
  double rrng[64], rrate[64], psrng[64], phrng[64], rate[64];
  float cnr[64];
  unsigned char info[64], half[64];
  int i, nsat, ncell, lock[64];

  /* encode msm header */
  if (!(i =
          encode_msm_head(7, rtcm, sys, sync, &nsat, &ncell, rrng, rrate, info,
                          psrng,
                          phrng, rate, lock, half, cnr)))
    return 0;

  /* encode msm satellite data */
  i = encode_msm_int_rrng(rtcm, i, rrng, nsat );  /* rough range integer ms */
  i = encode_msm_info(rtcm, i, info, nsat );      /* extended satellite info */
  i = encode_msm_mod_rrng(rtcm, i, rrng, nsat );  /* rough range modulo 1 ms */
  i = encode_msm_rrate(rtcm, i, rrate, nsat );    /* rough phase-range-rate */

  /* encode msm signal data */
  i = encode_msm_psrng_ex(rtcm, i, psrng, ncell); /* fine pseudorange ext */
  i = encode_msm_phrng_ex(rtcm, i, phrng, ncell); /* fine phase-range ext */
  i = encode_msm_lock_ex(rtcm, i, lock, ncell);   /* lock-time indicator ext */
  i = encode_msm_half_amb(rtcm, i, half, ncell);  /* half-cycle-amb indicator */
  i = encode_msm_cnr_ex(rtcm, i, cnr, ncell);     /* signal cnr ext */
  i = encode_msm_rate(rtcm, i, rate, ncell);      /* fine phase-range-rate */
  rtcm->nbit = i;
  return 1;
}
#endif

/* encode rtcm ver.3 message -------------------------------------------------*/
extern int encode_rtcm3(rtcm_t *rtcm, int type, int sync)
{
  int ret = 0;

  switch (type) {
  case 1002: ret = encode_type1002(rtcm, sync); break;

  case 1019: ret = encode_type1019(rtcm, sync); break;

  /*case 1077: ret=encode_msm7(rtcm,SYS_GPS,sync); break;*/
  default:
    ret = -1;
  }
  return ret;
}



/* generate rtcm 3 message -----------------------------------------------------
* generate rtcm 3 message
* args   : rtcm_t *rtcm   IO rtcm control struct
*          int    type    I  message type
*          int    sync    I  sync flag (1:another message follows)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int gen_rtcm3(rtcm_t *rtcm, int type, int sync)
{
  unsigned int crc;
  int i = 0;

  rtcm->nbit = rtcm->len = rtcm->nbyte = 0;

  /* set preamble and reserved */
  setbitu(rtcm->buff, i, 8, RTCM3PREAMB); i += 8;
  setbitu(rtcm->buff, i, 6, 0          ); i += 6;
  setbitu(rtcm->buff, i, 10, 0          ); i += 10;

  /* encode rtcm 3 message body */
  if (!encode_rtcm3(rtcm, type, sync)) return 0;

  /* padding to align 8 bit boundary */
  for (i = rtcm->nbit; i % 8; i++)
    setbitu(rtcm->buff, i, 1, 0);
  /* message length (header+data) (bytes) */
  if ((rtcm->len = i / 8) >= 3 + 1024) {
    /*trace(2,"generate rtcm 3 message length error len=%d\n",rtcm->len-3);*/
    rtcm->nbit = rtcm->len = 0;
    return 0;
  }
  /* message length without header and parity */
  setbitu(rtcm->buff, 14, 10, rtcm->len - 3);

  /* crc-24q */
  crc = crc24q(rtcm->buff, rtcm->len, 0);
  setbitu(rtcm->buff, i, 24, crc);

  /* length total (bytes) */
  rtcm->nbyte = rtcm->len + 3;

  return 1;
}

