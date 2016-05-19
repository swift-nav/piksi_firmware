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

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>

#include "nap_hw.h"
#include "nap_constants.h"

/** \addtogroup nap
 * \{ */

/** \defgroup conf Configuration
 * Functions to get information about the SwiftNAP configuration.
 * \{ */

u32 nap_conf_rd_version(void)
{
  NAP->CONTROL = (NAP->CONTROL & ~((u32)NAP_CONTROL_VERSION_ADDR_Msk));
  return NAP->VERSION;
}

u8 nap_conf_rd_version_string(char version_string[])
{
  u8 i = 0;
  u32 reg = 0;
  u32 ctrl = (NAP->CONTROL & ~((u32)NAP_CONTROL_VERSION_ADDR_Msk));

  do {
    NAP->CONTROL = ctrl | ((i + NAP_VERSION_STRING_OFFSET) / sizeof(reg) <<
        NAP_CONTROL_VERSION_ADDR_Pos);
    reg = NAP->VERSION;
    memcpy(&version_string[i], &reg, sizeof(reg));
    i += sizeof(reg);
  } while (reg && i < NAP_VERSION_STRING_LENGTH);
  version_string[i] = 0;

  return strlen(version_string);
}

void nap_rd_dna(u8 dna[])
{
  u32 reg = 0;
  u32 ctrl = (NAP->CONTROL & ~((u32)NAP_CONTROL_VERSION_ADDR_Msk));

  for (u8 i = 0; i < NAP_DNA_LENGTH; i += sizeof(reg)) {
    NAP->CONTROL = ctrl | ((i + NAP_DNA_OFFSET) / sizeof(reg) <<
        NAP_CONTROL_VERSION_ADDR_Pos);
    reg = NAP->VERSION;
    memcpy(&dna[i], &reg, sizeof(reg));
  }
}

void nap_unlock(const u8 key[])
{
  u32 ctrl = (NAP->CONTROL & ~((u32)NAP_CONTROL_KEY_ADDR_Msk |
        NAP_CONTROL_KEY_BYTE_Msk));

  for (u8 i = 0; i < NAP_KEY_LENGTH; ++i) {
    NAP->CONTROL = ctrl | ((u32)key[i] << NAP_CONTROL_KEY_BYTE_Pos) |
        (i << NAP_CONTROL_KEY_ADDR_Pos);
  }
}

/** \} */

/** \} */

