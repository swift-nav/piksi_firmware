/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_IONO_H_
#define SRC_IONO_H_

#include <libswiftnav/common.h>
#include <libswiftnav/ionosphere.h>

void gps_iono_params_store(const ionosphere_t *params);
u8 gps_iono_params_read(ionosphere_t *params);

#endif /* SRC_IONO_H_ */
