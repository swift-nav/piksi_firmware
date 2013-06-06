/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 * Copyright (C) 2013 Colin Beighley <colinbeighley@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "acq_channel.h"
#include "nap_common.h"

#include <libswiftnav/prns.h>

/*
 * Number of acquisition channel code phase taps that NAP configuration was
 * built with - read from configuration flash at runtime with nap_setup().
 */
u8 ACQ_N_TAPS;

/** Set the LOAD ENABLE bit of the NAP acquisition channel's LOAD register.
 * When the LOAD ENABLE bit is set, the acquisition channel will start loading
 * samples into its sample ram, starting at the first clock cycle after the
 * NAP's internal timing strobe goes low.
 */
void acq_set_load_enable_blocking()
{
  u8 temp[1] = {0xFF};
  nap_xfer_blocking(NAP_REG_ACQ_LOAD, 1, 0, temp);
}

/** Clear the LOAD ENABLE bit of the NAP acquisition channel's LOAD register.
 * After a load to the acquisition channel's sample ram, the LOAD ENABLE bit
 * must be cleared, or future timing strobes will cause the ram to be re-loaded.
 */
void acq_clear_load_enable_blocking()
{
  u8 temp[1] = {0x00};
  nap_xfer_blocking(NAP_REG_ACQ_LOAD, 1, 0, temp);
}

/** Pack data for writing to NAP acquisition channel INIT register.
 *
 * NOTE: Swift NAP returns corrs corresponding to code phases from
 * code_phase_reg_value-ACQ_N_TAPS-1 to code_phase_reg_value where
 * code_phase_reg_value is the raw value written into the code phase
 * portion of the init register.
 *
 * <ul>
 *   <li> corrs[0] -> code_phase_reg_value-ACQ_N_TAPS+1
 *   <li> corrs[AQC_N_TAPS-1] -> code_phase_reg_value
 * </ul>
 *
 * Lets take account of this here by writing code_phase+ACQ_N_TAPS-1
 * to the code phase register on the Swift NAP. This means the
 * correlations returned will be:
 *
 * <ul>
 *   <li> corrs[0] -> code_phase
 *   <li> corrs[ACQ_N_TAPS] -> code_phase-ACQ_N_TAPS+1
 * </ul>
 *
 * \param prn          PRN number - (0..31) (deprecated)
 * \param code_phase   Code phase of the first correlation returned
 *                     (see note above), in acquisition units.
 * \param carrier_freq Carrier frequency i.e. Doppler in acquisition
 *                     units.
 */
void acq_pack_init(u8 pack[], u8 prn, u16 code_phase, s16 carrier_freq)
{
  /* Modulo 1023*4 in case adding ACQ_N_TAPS-1 rolls us over a
   * code phase boundary.
   */
  u16 code_phase_reg_value = (code_phase+ACQ_N_TAPS-1) % (1023*4);

  pack[0] = (1<<5) |                      /* Acq enabled */
            ((carrier_freq >> 7) & 0x1F); /* Carrier freq [11:7] */

  pack[1] = (carrier_freq << 1) |         /* Carrier freq [6:0] */
            (code_phase_reg_value >> 11); /* Code phase [11] */

  pack[2] = code_phase_reg_value >> 3;    /* Code phase [10:3] */

  pack[3] = (code_phase_reg_value << 5) | /* Code phase [2:0] */
            (prn & 0x1F);                 /* PRN number (0..31) */
}

/** Write acquisition parameters to NAP acquisition channel's INIT register.
 * If the channel is currently disabled, it will be enabled and start an
 * acquisition with these parameters. If it is currently enabled, another
 * acquisition will start with these parameters when the current acquisition
 * finishes.
 * NOTE: If searching more than one acquisition point for a particular PRN,
 * the second set of acquisition parameters should be written into the channel
 * as soon as possible after the first set, as they are pipelined and used
 * immediately after the first acquisition finishes. If only searching one
 * point, acq_disable_blocking should be called as soon as possible after the
 * first set of acquisition parameters are written, and again after the
 * ACQ_DONE interrupt occurs to clear the interrupt.
 *
 * \param PRN          C/A PRN to use for the acquisition (deprecated)
 * \param code_phase   Code phase of the first correlation returned
 * \param carrier_freq Carrier frequency i.e. Doppler in acquisition units.
 */
/* TODO : remove writing of PRN number to init register */
void acq_write_init_blocking(u8 prn, u16 code_phase, s16 carrier_freq)
{
  u8 temp[4];
  acq_pack_init(temp, prn, code_phase, carrier_freq);
  nap_xfer_blocking(NAP_REG_ACQ_INIT, 4, 0, temp);
}

/** Disable NAP acquisition channel.
 * Write to the acquisition channel's INIT register to disable the correlations
 * (clears enable bit). This must be written once to pipeline disable the
 * correlations, and then a second time to clear the ACQ_DONE IRQ after the
 * last correlation has finished.
 */
void acq_disable_blocking()
{
  u8 temp[4] = {0,0,0,0};
  nap_xfer_blocking(NAP_REG_ACQ_INIT, 4, 0, temp);
}

/** Unpack correlations read from acquisition channel.
 *
 * \param packed Array of u8 data read from NAP acq channel CORR register.
 * \param corrs  Array of corr_t's of length ACQ_N_TAPS.
 */
void acq_unpack_corr(u8 packed[], corr_t corrs[])
{
  /* graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend */
  struct {s32 xtend:24;} sign;

  for (u8 i=0; i<ACQ_N_TAPS; i++) {

    sign.xtend  = (packed[6*i]   << 16)    /* MSB */
                | (packed[6*i+1] << 8)     /* Middle byte */
                | (packed[6*i+2]);         /* LSB */

    corrs[i].Q = sign.xtend; /* Sign extend! */

    sign.xtend  = (packed[6*i+3] << 16)    /* MSB */
                | (packed[6*i+4] << 8)     /* Middle byte */
                | (packed[6*i+5]);         /* LSB */

    corrs[i].I = sign.xtend; /* Sign extend! */
  }
}

/** Read correlations from acquisition channel.
 * Must be called after the NAP IRQ register IRQ_ACQ_DONE bit goes high, before
 * the next acquisition cycle is complete.
 *
 * \param corrs Array of corr_t of length ACQ_N_TAPS.
 */
void acq_read_corr_blocking(corr_t corrs[])
{
  u8 temp[2*ACQ_N_TAPS * 3];
  nap_xfer_blocking(NAP_REG_ACQ_CORR, 2*ACQ_N_TAPS*3, temp, temp);
  acq_unpack_corr(temp, corrs);
}

/** Write CA code to acquisition channel's code ram.
 * CA Code for SV to be searched for must be written into channel's code ram
 * before acquisitions are started.
 *
 * \param prn PRN number (0-31) of CA code to be written.
 */
void acq_write_code_blocking(u8 prn)
{
  nap_xfer_blocking(NAP_REG_ACQ_CODE, 128, 0, ca_code(prn));
}
