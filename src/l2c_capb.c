/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "l2c_capb.h"

/* Initial value matches the status of GPS constellation on 2016-05-17 */
static u32 gps_l2cm_l2c_capabilities = 0xf7814bfd;

void gps_l2cm_l2c_cap_store(u32 l2c_capb)
{
  gps_l2cm_l2c_capabilities = l2c_capb;
}

u32 gps_l2cm_l2c_cap_read()
{
  return gps_l2cm_l2c_capabilities;
}
