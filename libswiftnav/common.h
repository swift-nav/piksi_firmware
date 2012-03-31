/*
 * Copyright (C) 2012 Henry Hallam <henry@swift-nav.com>
 * Copyright (C) 2012 Fergus Noble <fergusnoble@gmail.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTLIB_COMMON_H
#define SWIFTLIB_COMMON_H

/** \defgroup lib SwiftNav Library
 *
 * Platform independent library of GNSS related functions used by the receiver.
 * This library implements GNSS related functions and algorithms for use by the
 * receiver and other software.
 *
 * It is intended to be as portable as possible and is written in standards
 * compliant C with no dependancies other than the standard C libraries. */

#ifndef LIBOPENCM3_CM3_COMMON_H
  /* Type definitions for shorter and nicer code */
  #include <stdint.h>
  #include <stdbool.h>
  typedef int8_t s8;
  typedef int16_t s16;
  typedef int32_t s32;
  typedef uint8_t u8;
  typedef uint16_t u16;
  typedef uint32_t u32;
  typedef uint64_t u64;
#endif

#endif /* SWIFTLIB_COMMON_H */

