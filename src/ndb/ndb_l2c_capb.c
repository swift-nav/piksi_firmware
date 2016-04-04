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

#define NDB_WEAK

#include <string.h>
#include <libswiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"

#define GPS_L2C_CAPB_FILE_NAME "l2c_capb"
static u32 gps_l2c_capabilities _CCM;
static ndb_element_metadata_t gps_l2c_capabilities_md _CCM;

static ndb_file_t gps_l2c_capb_file = {
  .name = GPS_L2C_CAPB_FILE_NAME,
  .fh = -1,
  .expected_size =
        sizeof(gps_l2c_capabilities)
      + sizeof(ndb_element_metadata_nv_t)
      + sizeof(ndb_file_end_mark),
  .data_size = sizeof(gps_l2c_capabilities),
  .n_elements = 1
};

void ndb_l2c_capb_init()
{
  ndb_load_data(&gps_l2c_capb_file, "l2c capabilities",
                &gps_l2c_capabilities, &gps_l2c_capabilities_md,
                 sizeof(gps_l2c_capabilities), 1);

  if (0 == gps_l2c_capabilities) {
    u32 new_val = 0xffffffff;
    ndb_update(&new_val, NDB_DS_INIT, &gps_l2c_capabilities_md);
    log_info("Save default L2C capability 0x%x", new_val);
  }
}

enum ndb_op_code ndb_gps_l2cm_l2c_cap_read(u32 *l2c_cap)
{
  ndb_retrieve(l2c_cap, &gps_l2c_capabilities, sizeof(gps_l2c_capabilities));
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_gps_l2cm_l2c_cap_store(u32 *l2c_cap,
                                            enum ndb_data_source src)
{
  return ndb_update(l2c_cap, src, &gps_l2c_capabilities_md);
}
