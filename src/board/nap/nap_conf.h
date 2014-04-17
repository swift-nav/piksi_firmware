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

/* DEPRECATED: These addresses now hold the version string but are still used
 * in the case where we detect and fall back to using old stlye versioning. */
#define NAP_FLASH_GIT_HASH_ADDR    0xE0000
#define NAP_FLASH_GIT_UNCLEAN_ADDR (NAP_FLASH_GIT_HASH_ADDR + 20)

/** \} */

void nap_conf_rd_parameters(void);

u8 nap_conf_rd_version_string(char version_string[]);

#endif /* SWIFTNAV_NAP_CONF_H */

