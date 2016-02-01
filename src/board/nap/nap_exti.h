/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_EXTI_H
#define SWIFTNAV_NAP_EXTI_H

#include <libswiftnav/common.h>


/** \addtogroup nap
 * \{ */

/* NAP IRQ register bit definitions. */
#define NAP_IRQ_ACQ_DONE      (1 << 31)
#define NAP_IRQ_ACQ_LOAD_DONE (1 << 30)
#define NAP_IRQ_CW_DONE       (1 << 29)
#define NAP_IRQ_CW_LOAD_DONE  (1 << 28)
#define NAP_IRQ_TIMING_STROBE (1 << 27)
#define NAP_IRQ_EXT_EVENT     (1 << 26)
#define NAP_IRQ_TRACK_MASK    (~(NAP_IRQ_ACQ_DONE | \
                                 NAP_IRQ_ACQ_LOAD_DONE | \
                                 NAP_IRQ_CW_DONE | \
                                 NAP_IRQ_CW_LOAD_DONE | \
				 NAP_IRQ_TIMING_STROBE | \
				 NAP_IRQ_EXT_EVENT ))

/** \} */

void nap_exti_setup(void);
u32 last_nap_exti_count(void);
void wait_for_nap_exti(void);

#endif  /* SWIFTNAV_NAP_EXTI_H */
