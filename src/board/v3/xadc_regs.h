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

#ifndef SWIFTNAV_XADC_REGS_H
#define SWIFTNAV_XADC_REGS_H

#include <stdint.h>

#define XADC_ADDR_RESET                   (0x03U)

#define XADC_ADDR_TEMPERATURE             (0x00U)
#define XADC_ADDR_VCCINT                  (0x01U)
#define XADC_ADDR_VCCAUX                  (0x02U)
#define XADC_ADDR_VPVN                    (0x03U)
#define XADC_ADDR_VREFP                   (0x04U)
#define XADC_ADDR_VREFN                   (0x05U)
#define XADC_ADDR_VCCBRAM                 (0x06U)

#define XADC_ADDR_VCCPINT                 (0x0DU)
#define XADC_ADDR_VCCPAUX                 (0x0EU)
#define XADC_ADDR_VCCODDR                 (0x0FU)

#define XADC_ADDR_CFG0                    (0x40U)
#define XADC_ADDR_CFG1                    (0x41U)
#define XADC_ADDR_CFG2                    (0x42U)

#define XADC_ADDR_ALARM_TEMP_UPPER        (0x50U)

#define XADC_ADDR_ALARM_OT_UPPER          (0x53U)
#define XADC_ADDR_ALARM_TEMP_LOWER        (0x54U)

#define XADC_ADDR_ALARM_OT_LOWER          (0x57U)

#endif /* SWIFTNAV_XADC_REGS_H */
