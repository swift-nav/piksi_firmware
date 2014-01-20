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
/* TODO: Change NAP version to be a string for more flexibility. */
#define NAP_FLASH_PARAMS_ADDR      0xD0000
#define NAP_FLASH_GIT_HASH_ADDR    0xE0000
#define NAP_FLASH_GIT_UNCLEAN_ADDR (NAP_FLASH_GIT_HASH_ADDR + 20)

/** \} */

void nap_conf_rd_parameters(void);

void nap_conf_rd_git_hash(u8 git_hash[]);
u8 nap_conf_rd_git_unclean(void);

#endif /* SWIFTNAV_NAP_CONF_H */

