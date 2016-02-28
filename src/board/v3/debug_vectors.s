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

    .text

    .global Und_Handler
Und_Handler:
    b Und_Handler

    .global Swi_Handler
Swi_Handler:
    b Swi_Handler

    .global Prefetch_Handler
Prefetch_Handler:
    b Prefetch_Handler

    .global Abort_Handler
Abort_Handler:
    b Abort_Handler

    .global Fiq_Handler
Fiq_Handler:
    b Fiq_Handler
