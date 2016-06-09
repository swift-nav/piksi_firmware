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

.macro handler str
    /* R1 = LR */
    mov r1, lr
    /* Switch to SYS mode with IRQs masked */
    msr CPSR_c, #0x9F
    /* R0 = str */
    ldr r0, =\str
    /* fault_handler_screaming_death(str, lr); */
    blx fault_handler_screaming_death
.endm

    .data

und_handler_str:
    .asciz "Undefined instruction!"
prefetch_handler_str:
    .asciz "Prefetch Handler!"
abort_handler_str:
    .asciz "Abort Handler!"
swi_handler_str:
    .asciz "Software Interrupt!"
fiq_handler_str:
    .asciz "Unused FIQ!"

    .text
    .code   32
    .balign 4

.global Und_Handler
Und_Handler:
    handler und_handler_str

.global Prefetch_Handler
Prefetch_Handler:
    handler prefetch_handler_str

.global Abort_Handler
Abort_Handler:
    handler abort_handler_str

.global Swi_Handler
Swi_Handler:
    handler swi_handler_str

.global Fiq_Handler
Fiq_Handler:
    handler fiq_handler_str

.end
