/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nap_common.h"

#include "nap_regs.h"
#include "nap_constants.h"
#include "axi_dma.h"

#include <math.h>

void nap_setup(void)
{
  axi_dma_init();
  axi_dma_start(&AXIDMADriver1);

  NAP->FE_PINC[0] = (u32)round(14.58e6 * pow(2.0, 32.0)
                                   / NAP_FRONTEND_SAMPLE_RATE_Hz);
}
