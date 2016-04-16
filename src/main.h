/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_MAIN_H
#define SWIFTNAV_MAIN_H

#include <libswiftnav/common.h>

/* See http://c-faq.com/cpp/multistmt.html for
 * and explaination of the do {} while(0)
 */
#define DO_EVERY(n, cmd) do { \
  static u32 do_every_count = 0; \
  if ((n) > 0 && do_every_count % (n) == 0) { \
    cmd; \
  } \
  do_every_count++; \
} while(0)

#define DO_ONLY(n, cmd) do { \
  static u32 do_only_count = 0; \
  if (do_only_count < (n)) { \
    do_only_count++; \
    cmd; \
  } \
} while(0)

/* See gcc.gnu.org/onlinedocs/cpp/Stringification.html for
 * explanation of pre-processing of macros into strings.
 */
#define STR_INNER(x) #x
#define STR(x) STR_INNER(x)

#endif
