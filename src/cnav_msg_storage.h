/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_CNAV_MSG_STORAGE_H
#define LIBSWIFTNAV_CNAV_MSG_STORAGE_H

#include <libswiftnav/cnav_msg.h>

void cnav_msg_type30_put(cnav_msg_t *msg);
void cnav_msg_type30_get(u8 prn, cnav_msg_type_30_t *msg);

#endif /* LIBSWIFTNAV_CNAV_MSG_STORAGE_H */
