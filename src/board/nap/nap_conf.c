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
    m25_read(NAP_FLASH_PARAMS_ADDR + i, 1, nap_parameters[i]);
}

/** Return git commit hash from NAP configuration build.
 * Retrieves commit hash of HDL repository at the time FPGA configuration was
 * built.
 *
 * \param git_hash Array of u8 (length 20) in which git hash will be put.
 */
void nap_conf_rd_git_hash(u8 git_hash[])
{
  m25_read(NAP_FLASH_GIT_HASH_ADDR, 20, git_hash);
}

/** Return git repository cleanliness status from NAP configuration build.
 * Retrieves cleanliness status of HDL repository at the time FPGA
 * configuration was built.
 *
 * \return 1 if repository was unclean, 0 if clean
 */
u8 nap_conf_rd_git_unclean(void)
{
  u8 unclean;

  m25_read(NAP_FLASH_GIT_UNCLEAN_ADDR, 1, &unclean);
  return unclean;
}

/** \} */

/** \} */

