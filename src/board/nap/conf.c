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

#include "conf.h"
#include "nap_common.h"
#include "track_channel.h"
#include "acq_channel.h"
#include "../m25_flash.h"

/** \addtogroup nap
 * \{ */

/** \defgroup conf Configuration
 * Functions to get information about the NAP configuration.
 * \{ */

/** Get NAP configuration parameters from FPGA configuration flash.
 * Gets information about the NAP configuration (number of code phase taps in
 * the acquisition channel, number of tracking channels, etc).
 */
void nap_conf_rd_parameters()
{
  /* Define parameters that need to be read from FPGA configuration flash.
   * Pointers in the array should be in the same order they're stored in the
   * configuration flash. */
  u8 * nap_parameters[2] = {
                            &nap_acq_n_taps,
                            &nap_track_n_channels
                           };
  /* Get parameters from FPGA configuration flash */
  for (u8 i=0; i<(sizeof(nap_parameters)/sizeof(nap_parameters[0])); i++){
    m25_read(NAP_FLASH_PARAMS_ADDR + i, 1, nap_parameters[i]);
  }
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
u8 nap_conf_rd_git_unclean()
{
  u8 unclean;
  m25_read(NAP_FLASH_GIT_UNCLEAN_ADDR, 1, &unclean);
  return unclean;
}

/** \} */

/** \} */
