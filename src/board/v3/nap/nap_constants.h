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

#ifndef SWIFTNAV_NAP_CONSTANTS_H
#define SWIFTNAV_NAP_CONSTANTS_H

#define NAP_FRONTEND_SAMPLE_RATE_Hz                                (99.375e6)

#define NAP_ACQ_DECIMATION_RATE                                          (12)
#define NAP_ACQ_SAMPLE_RATE_Hz                  (NAP_FRONTEND_SAMPLE_RATE_Hz  \
                                                   / NAP_ACQ_DECIMATION_RATE)

#endif /* SWIFTNAV_NAP_CONSTANTS_H */
