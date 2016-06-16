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

#include <assert.h>
#include <string.h>
#include <ch.h>

#include "iono.h"
#include "signal.h"

MUTEX_DECL(iono_mutex);

static ionosphere_t iono_params;
static bool iono_params_decoded_flag = false;

/** Stores ionospheric parameters
 * \param params pointer to ionospheric parameters to be stored
 */
void gps_iono_params_store(ionosphere_t *params)
{
  assert(params != NULL);
  chMtxLock(&iono_mutex);
  memcpy(&iono_params, params, sizeof(ionosphere_t));
  iono_params_decoded_flag = true;
  chMtxUnlock(&iono_mutex);
}

/** Reads ionospheric parameters
 * \param params pointer to ionospheric
 * \return 1 if iono parameters available otherwise 0
 */
u8 gps_iono_params_read(ionosphere_t *params)
{
  assert(params != NULL);
  chMtxLock(&iono_mutex);
  if (iono_params_decoded_flag) {
    memcpy(params, &iono_params, sizeof(ionosphere_t));
    chMtxUnlock(&iono_mutex);
    return 1;
  } else {
    chMtxUnlock(&iono_mutex);
    return 0;
  }
}
