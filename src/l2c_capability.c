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
#include <stdio.h>
#include <ch.h>
#include <assert.h>

#include <libswiftnav/common.h>
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/logging.h>

#include "cfs/cfs-coffee.h"
#include "cfs/cfs.h"

#include "l2c_capability.h"

static MUTEX_DECL(l2c_capability_upd_mutex);

/** gps_l2c_sv_capability 
  L2C capability info containter
  Each bit responses for a SV:
  Bit 0 - SV 1, Bit 31 - SV 32.
  bit set - L2C stream available for the SV
  bit cleared - L2C stream not available
  By default assume all SV can broadcast L2C stream */
static u32 gps_l2c_sv_capability = 0xffffffff;

/** \defgroup l2c_capability L2C_Capability
 * Functions used in l2c capability
 * \{ */

/** Update gps_l2c_sv_capability. And write it to Flash memory if
 * previous value of gps_l2c_sv_capability is different.
 * \param l2c_cpbl New value of L2C capability 
 *
 */
void l2c_capability_update(u32 l2c_cpbl)
{
  if (l2c_cpbl == gps_l2c_sv_capability)
    return;

  chMtxLock(&l2c_capability_upd_mutex);

  gps_l2c_sv_capability = l2c_cpbl;

  int fd = cfs_open(L2C_FILE, CFS_WRITE);
  if (fd != -1) {
    if(cfs_write(fd,
              (void *)&gps_l2c_sv_capability,
              sizeof(gps_l2c_sv_capability)) ==
              sizeof(gps_l2c_sv_capability))
      log_info("L2C capability updated, new value is 0x%x",
                gps_l2c_sv_capability);
    else
      log_error("L2C capability write error");

    cfs_close(fd);
  } else
    log_error("Error opening L2C capability file");

  Mutex *m = chMtxUnlock();
  assert(m == &l2c_capability_upd_mutex);
}

/** Return current value of gps_l2c_sv_capability
 *  \param l2c_capbl pointer to output value of L2C capability
 *
 *  \return 0 if L2C value was read successfully, otherwise -1
 */
s8 l2c_capability_get(u32 *l2c_cpbl)
{
  if (NULL == l2c_cpbl)
    return -1;
  chMtxLock(&l2c_capability_upd_mutex);
  *l2c_cpbl = gps_l2c_sv_capability;
  Mutex *m = chMtxUnlock();
  assert(m == &l2c_capability_upd_mutex);
  return 0;
}

/** Init gps_l2c_sv_capability.
 * Sets initial value of L2C capability and write it in a file, creates
 * one if not exist
 */
void l2c_capability_init()
{ 
  chMtxLock(&l2c_capability_upd_mutex);

  int fd = cfs_open(L2C_FILE,CFS_READ);

  if (fd != -1) {
    /* read previous stored value of L2C capability */
    if (cfs_read(fd, &gps_l2c_sv_capability,
          sizeof(gps_l2c_sv_capability)) ==
          sizeof(gps_l2c_sv_capability))
      log_info("Loaded last L2C capability from file: 0x%x",
                gps_l2c_sv_capability);
    else
      log_warn("Cannot load last L2C capability from file");

    cfs_close(fd);
  } else {
    /* No L2C capability file present in flash, create an empty one and
     * init by default value */
    log_info("No L2C capability file present in flash, create an empty one");
    cfs_coffee_reserve(L2C_FILE, sizeof(gps_l2c_sv_capability));
    cfs_coffee_configure_log(L2C_FILE, 256, sizeof(gps_l2c_sv_capability));
    int fd = cfs_open(L2C_FILE,CFS_WRITE);

    if (cfs_write(fd, (void *)&gps_l2c_sv_capability,
          sizeof(gps_l2c_sv_capability)) !=
          sizeof(gps_l2c_sv_capability))
      log_error("Error writing to L2C capability file");
    else
      log_info("Created L2C capability file and saved default value: 0x%x",
               gps_l2c_sv_capability);

    cfs_close(fd);
  }

  Mutex *m = chMtxUnlock();
  assert(m == &l2c_capability_upd_mutex);
}

/** \} */
