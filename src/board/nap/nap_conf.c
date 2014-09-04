/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
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

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>

#include "../m25_flash.h"
#include "acq_channel.h"
#include "nap_conf.h"
#include "nap_common.h"
#include "track_channel.h"

/** \addtogroup nap
 * \{ */

/** \defgroup conf Configuration
 * Functions to get information about the SwiftNAP configuration.
 * \{ */

/** Get NAP configuration parameters from FPGA configuration flash.
 * Gets information about the NAP configuration (number of code phase taps in
 * the acquisition channel, number of tracking channels, etc).
 */
void nap_conf_rd_parameters(void)
{
  /* Define parameters that need to be read from FPGA configuration flash.
   * Pointers in the array should be in the same order they're stored in the
   * configuration flash. */
  u8 * nap_parameters[] = {
    &nap_acq_fft_index_bits,
    &nap_acq_downsample_stages,
    &nap_track_n_channels
  };

  /* Get parameters from FPGA configuration flash */
  for (u8 i = 0; i < (sizeof(nap_parameters) / sizeof(nap_parameters[0])); i++)
    m25_read(NAP_FLASH_PARAMS_ADDR + i, nap_parameters[i], 1);

  /* Bound number of channels with used by libswiftnav MAX_CHANNELS parameter. */
  nap_track_n_channels = MIN(nap_track_n_channels, MAX_CHANNELS);
}

/** Return version string from NAP configuration build.
 *
 * \param git_hash char [] that version string will be written to.
 * \return Length of version string.
 */
u8 nap_conf_rd_version_string(char version_string[])
{
  u8 count = 0;
  char c;

  do {
    m25_read(NAP_FLASH_VERSION_STRING_ADDR + count, (u8 *)&c, 1);
    version_string[count] = c;
    count++;

    if (c && !isprint((u8)c)) {
      /* We have hit an unexpected character, this must not be an ASCII version
       * string. Fall back to old Git Hash style version. */

      strcpy(version_string, "OLD ");
      for (count=0; count<20; count++) {
        m25_read(NAP_FLASH_GIT_HASH_ADDR + count, (u8 *)&c, 1);
        snprintf(&version_string[2*count + 4], 3, "%02x", c);
      }
      u8 unclean;
      m25_read(NAP_FLASH_GIT_UNCLEAN_ADDR, &unclean, 1);
      if (unclean) {
        strcpy(&version_string[44], " (unclean)");
      }
      count = strlen(version_string);
      /* Make sure we exit the loop. */
      break;
    }
  } while (c);

  return count;
}

/** Return Piksi serial number from the configuration flash.
 */
s32 nap_conf_rd_serial_number()
{
  u8 serial_num_u8[4];
  m25_read(NAP_FLASH_SERIAL_NUMBER_ADDR, serial_num_u8, 4);
  s32 serial_num = (serial_num_u8[0] << 24) |
                   (serial_num_u8[1] << 16) |
                   (serial_num_u8[2] << 8) |
                   (serial_num_u8[3] << 0);
  return serial_num;
}

/** Return Piksi hardware revision identifier.
 */
u32 nap_conf_rd_hw_rev()
{
  u8 hw_rev_u8[4];
  m25_read(NAP_FLASH_HW_REVISION_ADDR, hw_rev_u8, 4);
  u32 hw_rev = (hw_rev_u8[0] << 24) |
               (hw_rev_u8[1] << 16) |
               (hw_rev_u8[2] << 8) |
               (hw_rev_u8[3] << 0);
  return hw_rev;
}

/** Return Piksi hardware revision string.
 */
const char * nap_conf_rd_hw_rev_string()
{
  const char *rev_strings[] = {
    "(unknown)",
    "piksi_2.3.1",
  };

  /* If hw_rev is equal to 0xFFFFFFFF (i.e. unprogrammed)
   * then hw_rev+1 == 0. */
  u32 hw_rev = nap_conf_rd_hw_rev()+1;
  if (hw_rev > sizeof(rev_strings)/sizeof(rev_strings[0]))
    hw_rev = 0;
  return rev_strings[hw_rev];
}

/** \} */

/** \} */

