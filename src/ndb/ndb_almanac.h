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

#ifndef SRC_NDB_ALMANAC_H_
#define SRC_NDB_ALMANAC_H_

#include <ndb/ndb_common.h>

void ndb_almanac_init();
enum ndb_op_code ndb_almanac_read(gnss_signal_t sid, almanac_t *a);
enum ndb_op_code ndb_almanac_store(almanac_t *a, enum ndb_data_source);
enum ndb_op_code ndb_almanac_cache_update(almanac_t *cached_a,
                                          ndb_update_counter_t *uc);
void ndb_almanac_sbp_update();
#endif /* SRC_NDB_ALMANAC_H_ */
