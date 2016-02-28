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
    .fpu    neon
    .code   32
    .balign 4

    .global __early_init
__early_init:

#if defined(THUMB_NO_INTERWORKING)
    .code   16
    mov r0, pc
    bx  r0
    .code   32
#endif

    /* Invalidate caches and TLBs */
    mov r0, #0                      /* r0 = 0  */
    mcr p15, 0, r0, c8, c7, 0       /* invalidate TLBs */
    mcr p15, 0, r0, c7, c5, 0       /* invalidate icache */
    mcr p15, 0, r0, c7, c5, 6       /* Invalidate branch predictor array */
    b invalidate_dcache             /* invalidate dcache */

invalidate_dcache_ret:

    /* Disable MMU */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    bic r0, r0, #0x1                /* clear M bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */

    /* Invalidate scu */
    ldr r7, =0xf8f0000c
    ldr r6, =0xffff
    str r6, [r7]

    /* Set scu enable bit */
    ldr r7, =0xf8f00000
    ldr r0, [r7]
    orr r0, r0, #0x1
    str r0, [r7]

    /* Set MMU TTB0 base */
    ldr r0, =MMUTable               /* Load MMU translation table base */
    orr r0, r0, #0x5B               /* Outer-cacheable, WB */
    mcr p15, 0, r0, c2, c0, 0       /* write TTB0 */

    /* Configure MMU domains */
    mov r0, #0xffffffff             /* all ones = manager, permissions ignored */
    mcr p15, 0, r0, c3, c0, 0       /* write DACR */

    /* Enable mmu, icahce and dcache */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    orr r0, r0, #(0x1 << 12)        /* set I bit */
    orr r0, r0, #(0x1 << 2)         /* set C bit */
    orr r0, r0, #(0x1 << 0)         /* set M bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */
    dsb                             /* dsb allow the MMU to start up */
    isb                             /* isb flush prefetch buffer */

    /* Write to ACTLR */
    mrc p15, 0, r0, c1, c0, 1       /* Read ACTLR */
    orr r0, r0, #(0x1 << 6)         /* set SMP bit */
    orr r0, r0, #(0x1 << 0)         /* set FW bit */
    mcr p15, 0, r0, c1, c0, 1       /* Write ACTLR */

    /* Allow full access to coprocessors */
    mrc p15, 0, r0, c1, c0, 2       /* read CPACR  */
    orr r0, r0, #(0xf << 20)        /* enable full access for p10 & p11 */
    mcr p15, 0, r0, c1, c0, 2       /* write CPACR */

    /* Enable vfp */
    fmrx r0, FPEXC                  /* read FPEXC */
    orr r0, r0, #(0x1 << 30)        /* set VFP enable bit */
    fmxr FPEXC, r0                  /* write FPEXC */

    /* Enable flow prediction */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    orr r0, r0, #(0x1 << 11)        /* set Z bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */

    /* Enable Dside prefetch and L2 prefetch hint */
    mrc p15, 0, r0, c1, c0, 1       /* read ACTLR */
    orr r0, r0, #(0x1 << 2)         /* enable Dside prefetch */
    orr r0, r0, #(0x1 << 1)         /* enable L2 Prefetch hint */
    mcr p15, 0, r0, c1, c0, 1       /* write ACTLR */

    /* Enable unaligned data access */
    mrc p15, 0, r0, c1, c0, 0       /* read SCTLR */
    bic r0, r0, #(0x1 << 1)         /* clear A bit */
    mcr p15, 0, r0, c1, c0, 0       /* write SCTLR */

    /* Unmask asynchronous abort exception */
    mrs r0, cpsr                    /* read CPSR */
    bic r0, r0, #(0x1 << 8)         /* clear A bit */
    msr cpsr_xsf, r0                /* write CPSR<31:8> */

    /* Done */
#if defined(THUMB_NO_INTERWORKING)
    add r0, pc, #1
    bx r0
    .code   16
    bx lr
    .code   32
#else
    bx lr
#endif

/*
 *************************************************************************
 *
 * invalidate_dcache - invalidate the entire d-cache by set/way
 *
 * Note: for Cortex-A9, there is no cp instruction for invalidating
 * the whole D-cache. Need to invalidate each line.
 *
 *************************************************************************
 */
invalidate_dcache:
    mrc p15, 1, r0, c0, c0, 1       /* read CLIDR */
    ands r3, r0, #0x7000000
    mov r3, r3, lsr #23             /* cache level value (naturally aligned) */
    beq finished
    mov r10, #0                     /* start with level 0 */
loop1:
    add r2, r10, r10, lsr #1        /* work out 3xcachelevel */
    mov r1, r0, lsr r2              /* bottom 3 bits are the Cache type for this level */
    and r1, r1, #7                  /* get those 3 bits alone */
    cmp r1, #2
    blt skip                        /* no cache or only instruction cache at this level */
    mcr p15, 2, r10, c0, c0, 0      /* write the Cache Size selection register */
    isb                             /* isb to sync the change to the CacheSizeID reg */
    mrc p15, 1, r1, c0, c0, 0       /* reads current Cache Size ID register */
    and r2, r1, #7                  /* extract the line length field */
    add r2, r2, #4                  /* add 4 for the line length offset (log2 16 bytes) */
    ldr r4, =0x3ff
    ands r4, r4, r1, lsr #3         /* r4 is the max number on the way size (right aligned) */
    clz r5, r4                      /* r5 is the bit position of the way size increment */
    ldr r7, =0x7fff
    ands r7, r7, r1, lsr #13        /* r7 is the max number of the index size (right aligned) */
loop2:
    mov r9, r4                      /* r9 working copy of the max way size (right aligned) */
loop3:
    orr r11, r10, r9, lsl r5        /* factor in the way number and cache number into r11 */
    orr r11, r11, r7, lsl r2        /* factor in the index number */
    mcr p15, 0, r11, c7, c6, 2      /* invalidate by set/way */
    subs r9, r9, #1                 /* decrement the way number */
    bge loop3
    subs r7, r7, #1                 /* decrement the index */
    bge loop2
skip:
    add r10, r10, #2                /* increment the cache number */
    cmp r3, r10
    bgt loop1

finished:
    mov r10, #0                     /* swith back to cache level 0 */
    mcr p15, 2, r10, c0, c0, 0      /* select current cache level in cssr */
    dsb
    isb
    b invalidate_dcache_ret

.end
