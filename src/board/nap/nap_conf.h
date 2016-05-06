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

#ifndef SWIFTNAV_NAP_CONF_H
#define SWIFTNAV_NAP_CONF_H

#include <libswiftnav/common.h>

/** \addtogroup conf
 * \{ */

/* Configuration flash addresses of interest. */
#define NAP_FLASH_PARAMS_ADDR         0xD0000
#define NAP_FLASH_VERSION_STRING_ADDR 0xE0000
#define NAP_FLASH_SERIAL_NUMBER_ADDR  0xF0000
#define NAP_FLASH_HW_REVISION_ADDR    0xF0004

/* DEPRECATED: These addresses now hold the version string but are still used
 * in the case where we detect and fall back to using old stlye versioning. */
#define NAP_FLASH_GIT_HASH_ADDR    0xE0000
#define NAP_FLASH_GIT_UNCLEAN_ADDR (NAP_FLASH_GIT_HASH_ADDR + 20)

/** \} */

extern u8 nap_acq_fft_index_bits;
extern u8 nap_acq_downsample_stages;

void nap_conf_rd_parameters(void);

u8 nap_conf_rd_version_string(char version_string[]);
u32 nap_conf_rd_version(void);
s32 nap_conf_rd_serial_number(void);
u32 nap_conf_rd_hw_rev(void);
const char * nap_conf_rd_hw_rev_string(void);

#endif /* SWIFTNAV_NAP_CONF_H */

