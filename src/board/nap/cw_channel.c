/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "cw_channel.h"
#include "nap_common.h"

/** \addtogroup nap
 * \{ */

/** \defgroup cw_channel CW Channel
 * Interface to the NAP CW channel.
 * \{ */

/** Set the LOAD ENABLE bit of the NAP CW channel's LOAD register.
 * When the LOAD ENABLE bit is set, the CW channel will start loading samples
 * into its sample ram, starting at the first sample clock cycle after the
 * NAP's internal timing strobe goes low. Writing to the LOAD register will
 * clear the CW_LOAD interrupt.
 */
void nap_cw_load_wr_enable_blocking(void)
{
  u8 temp[1] = { 0xFF };

  nap_xfer_blocking(NAP_REG_CW_LOAD, 1, 0, temp);
}

/** Clear the LOAD ENABLE bit of the NAP CW channel's LOAD register.
 * After a load to the CW channel's sample ram, the LOAD ENABLE bit must be
 * cleared, or future timing strobes will cause the ram to be re-loaded.
 * Writing to the LOAD register will clear the CW_LOAD interrupt.
 */
void nap_cw_load_wr_disable_blocking(void)
{
  u8 temp[1] = { 0x00 };

  nap_xfer_blocking(NAP_REG_CW_LOAD, 1, 0, temp);
}

/** Pack data for writing to NAP CW channel INIT register.
 *
 * \param prn          PRN number - (0..31) (deprecated)
 * \param carrier_freq CW frequency in CW INIT register units.
 */
void nap_cw_init_pack(u8 pack[], s32 carrier_freq)
{
  pack[0] = (1 << 3) |                      /* cw enabled */
            ((carrier_freq >> 30) & 0x04) | /* carrier freq [sign] */
            ((carrier_freq >> 16) & 0x03);  /* carrier freq [17:16] */
  pack[1] = (carrier_freq >> 8) & 0xFF;     /* carrier freq [15:8] */
  pack[2] = carrier_freq & 0xFF;            /* carrier freq [7:0] */
}

/** Write CW parameters to NAP CW channel's INIT register.
 * If the channel is currently disabled, it will be enabled and start a CW
 * search with these parameters. If it is currently enabled, another CW search
 * will start with these parameters when the current CW search finishes.
 *
 * \note If searching more than one spectrum point, the second set of CW search
 *       parameters should be written into the channel as soon as possible
 *       after the first set, as they are pipelined and used immediately after
 *       the first CW correlation finishes. If only searching one point,
 *       nap_cw_init_wr_disable_blocking should be called as soon as possible
 *       after the first set of CW parameters are written, and again after the
 *       CW_DONE interrupt occurs to clear the interrupt.
 *
 * \param carrier_freq CW frequency in CW INIT register units.
 */
void nap_cw_init_wr_params_blocking(s32 carrier_freq)
{
  u8 temp[3];

  nap_cw_init_pack(temp, carrier_freq);
  nap_xfer_blocking(NAP_REG_CW_INIT, 3, 0, temp);
}

/** Disable NAP CW channel.
 * Write to the CW channel's INIT register to disable the correlations (clears
 * enable bit). This must be written once to pipeline disable the correlations,
 * and then a second time to clear the CW_DONE IRQ after the last correlation
 * has finished.
 */
void nap_cw_init_wr_disable_blocking(void)
{
  u8 temp[3] = { 0, 0, 0 };

  nap_xfer_blocking(NAP_REG_CW_INIT, 3, 0, temp);
}

/** Unpack correlations read from CW channel.
 *
 * \param packed Array of u8 data read from NAP CW channel CORR register.
 * \param corrs  Pointer to single corr_t.
 */
void nap_cw_corr_unpack(u8 packed[], corr_t* corrs)
{
  /* http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */

  struct { s32 xtend : 24; } sign;

  sign.xtend  = (packed[0] << 16) /* MSB */
              | (packed[1] << 8)  /* Middle byte */
              | (packed[2]);      /* LSB */

  corrs->Q = sign.xtend; /* Sign extend! */

  sign.xtend  = (packed[3] << 16) /* MSB */
              | (packed[4] << 8)  /* Middle byte */
              | (packed[5]);      /* LSB */

  corrs->I = sign.xtend; /* Sign extend! */
}

/** Read correlations from CW channel.
 * Must be called after the NAP IRQ register IRQ_CW_DONE bit goes high, before
 * the next CW cycle is complete.
 *
 * \param corrs Pointer to single corr_t.
 */
void nap_cw_corr_rd_blocking(corr_t* corrs)
{
  u8 temp[6]; /* 6 u8 = 48 bits = 2*(24 bits) */

  nap_cw_corr_unpack(temp, corrs);
  nap_xfer_blocking(NAP_REG_CW_CORR, 6, temp, temp);
}

/** \} */

/** \} */

