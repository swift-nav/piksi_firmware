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

#ifndef SRC_NDB_EPHEMERIS_H_
#define SRC_NDB_EPHEMERIS_H_

#include <ndb/ndb_common.h>

void ndb_ephemeris_init();
enum ndb_op_code ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e) NDB_WEAK;
enum ndb_op_code ndb_ephemeris_store(ephemeris_t *e,
                                     enum ndb_data_source) NDB_WEAK;
enum ndb_op_code ndb_ephemeris_info(gnss_signal_t sid, u8* v, u8* h,
                                    gps_time_t* toe,
                                    u32* fit_interval) NDB_WEAK;
enum ndb_op_code ndb_ephemeris_cache_update(ephemeris_t *cached_e,
                                            ndb_update_counter_t *uc) NDB_WEAK;

void ndb_ephemeris_sbp_update();
#endif /* SRC_NDB_EPHEMERIS_H_ */
