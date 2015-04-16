/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_ERROR_H
#define SWIFTNAV_ERROR_H

/* Two macros ensures any macro passed will
 * be expanded before being stringified */
#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)

#define screaming_death(x) _screaming_death(__FILE__ ":" STRINGIZE(__LINE__), (x))
__attribute__((noreturn)) void _screaming_death(const char *pos, const char *msg);

void fault_handling_setup(void);


#endif  /* SWIFTNAV_ERROR_H */

