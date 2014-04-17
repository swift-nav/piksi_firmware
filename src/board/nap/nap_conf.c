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
  u8 * nap_parameters[2] = {
    &nap_acq_n_taps,
    &nap_track_n_channels
  };

  /* Get parameters from FPGA configuration flash */
  for (u8 i = 0; i < (sizeof(nap_parameters) / sizeof(nap_parameters[0])); i++)
    m25_read(NAP_FLASH_PARAMS_ADDR + i, nap_parameters[i], 1);
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

  m25_read(NAP_FLASH_VERSION_STRING_ADDR, (u8 *)&c, 1);
  while (c) {
    version_string[count] = c;
    count++;
    m25_read(NAP_FLASH_VERSION_STRING_ADDR + count, (u8 *)&c, 1);
  }

  /* Append 0 for proper string delimitation. */
  version_string[count] = 0;
  count++;

  return count;
}

/** \} */

/** \} */

