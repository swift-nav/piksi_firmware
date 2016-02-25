/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_FRONTEND_H
#define SWIFTNAV_FRONTEND_H

#include <libswiftnav/common.h>

typedef enum {
  AUTO,
  PATCH,
  EXTERNAL,
  EXTERNAL_AC
} antenna_type_t;

void frontend_configure(void);
void frontend_setup(void);
bool frontend_ant_status(void);
antenna_type_t frontend_ant_setting(void);

#endif

