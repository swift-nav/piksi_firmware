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

/** \} */

u32 nap_conf_rd_version(void);
u8 nap_conf_rd_version_string(char version_string[]);
void nap_rd_dna(u8 dna[]);
void nap_unlock(const u8 key[]);

#endif /* SWIFTNAV_NAP_CONF_H */

