/*
 * Copyright (C) 2012 Colin Beighley <colinbeighley@gmail.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTLIB_PRNS_H
#define SWIFTLIB_PRNS_H

#include "common.h"

const u8* ca_code(u8 prn);
inline s8 get_chip(u8* code, u32 chip_num);

#endif /* SWIFTLIB_PRNS_H */
