/*
 * Copyright (C) 2011-2013 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_MAX2769_H
#define SWIFTNAV_MAX2769_H

#include <libopencm3/cm3/common.h>

/** \addtogroup max
 * \{ */

#define MAX2769_CONF1   0x00
#define MAX2769_CONF2   0x01
#define MAX2769_CONF3   0x02
#define MAX2769_PLLCONF 0x03
#define MAX2769_DIV     0x04
#define MAX2769_FDIV    0x05
#define MAX2769_STRM    0x06
#define MAX2769_CLK     0x07
#define MAX2769_TEST1   0x08
#define MAX2769_TEST2   0x09

/* MAX2769 Configuration 1. */

#define MAX2769_CONF1_CHIPEN        (1<<27)
#define MAX2769_CONF1_IDLE          (1<<26)
#define MAX2769_CONF1_ILNA1(n)      ((n & 0xF)<<22)
#define MAX2769_CONF1_ILNA2(n)      ((n & 0x3)<<20)
#define MAX2769_CONF1_ILO(n)        ((n & 0x3)<<18)
#define MAX2769_CONF1_IMIX(n)       ((n & 0x3)<<16)

#define MAX2769_CONF1_MIXPOLE_36MHZ (1<<15)
#define MAX2769_CONF1_MIXPOLE_13MHZ (0<<15)

#define MAX2769_CONF1_LNAMODE_GATED (0<<13)
#define MAX2769_CONF1_LNAMODE_LNA2  (1<<13)
#define MAX2769_CONF1_LNAMODE_LNA1  (2<<13)
#define MAX2769_CONF1_LNAMODE_OFF   (3<<13)

#define MAX2769_CONF1_MIXEN         (1<<12)
#define MAX2769_CONF1_ANTEN         (1<<11)
#define MAX2769_CONF1_FCEN(n)       ((n & 0x3F)<<5)

#define MAX2769_CONF1_FBW_2_5MHZ    (0<<3)
#define MAX2769_CONF1_FBW_4_2MHZ    (2<<3)
#define MAX2769_CONF1_FBW_8MHZ      (1<<3)
#define MAX2769_CONF1_FBW_18MHZ     (3<<3)

#define MAX2769_CONF1_F3OR5_5       (0<<2)
#define MAX2769_CONF1_F3OR5_3       (1<<2)

#define MAX2769_CONF1_FCENX_BP      (1<<1)
#define MAX2769_CONF1_FCENX_LP      (0<<1)

#define MAX2769_CONF1_FGAIN_HIGH    (1<<0)
#define MAX2769_CONF1_FGAIN_LOW     (0<<0)

/* MAX2769 Configuration 2. */

#define MAX2769_CONF2_IQEN            (1<<27)
#define MAX2769_CONF2_GAINREF(n)      ((n & 0xFFF)<<15)
/* CONF2 bits 14:13 reserved. */

#define MAX2769_CONF2_AGCMODE_INDEP   (0<<11)
#define MAX2769_CONF2_AGCMODE_LOCKED  (1<<11)
#define MAX2769_CONF2_AGCMODE_GAININ  (2<<11)

#define MAX2769_CONF2_FORMAT_UNSIGNED (0<<9)
#define MAX2769_CONF2_FORMAT_SIGN_MAG (1<<9)
#define MAX2769_CONF2_FORMAT_2S_COMP  (2<<9)

#define MAX2769_CONF2_BITS_1          (0<<6)
#define MAX2769_CONF2_BITS_1_5        (1<<6)
#define MAX2769_CONF2_BITS_2          (2<<6)
#define MAX2769_CONF2_BITS_2_5        (3<<6)
#define MAX2769_CONF2_BITS_3          (4<<6)

#define MAX2769_CONF2_DRVCFG_CMOS     (0<<4)
#define MAX2769_CONF2_DRVCFG_DIFF     (1<<4)
#define MAX2769_CONF2_DRVCFG_ANALOG   (2<<4)

#define MAX2769_CONF2_LOEN            (1<<3)
/* CONF2 bit 2 reserved. */
/* MAX2769_CONF2_DIEID not implemented, write only!? */

/* MAX2769 Configuration 3. */

#define MAX2769_CONF3_GAININ(n)    ((n & 0x3F)<<22)
#define MAX2769_CONF3_FSLOWEN      (1<<21)
#define MAX2769_CONF3_HILOADEN     (1<<20)
#define MAX2769_CONF3_ADCEN        (1<<19)
#define MAX2769_CONF3_DRVEN        (1<<18)
#define MAX2769_CONF3_FOFSTEN      (1<<17)
#define MAX2769_CONF3_FILTEN       (1<<16)
#define MAX2769_CONF3_FHIPEN       (1<<15)
/* CONF3 bit 14 reserved. */
#define MAX2769_CONF3_PGAIEN       (1<<13)
#define MAX2769_CONF3_PGAQEN       (1<<12)
#define MAX2769_CONF3_STRMEN       (1<<11)
#define MAX2769_CONF3_STRMSTART    (1<<10)
#define MAX2769_CONF3_STRMSTOP     (1<<9)
#define MAX2769_CONF3_STRMCOUNT(n) ((n & 0x7)<<6)
#define MAX2769_CONF3_STRMBITS(n)  ((n & 0x3)<<4)
#define MAX2769_CONF3_STAMPEN      (1<<3)
#define MAX2769_CONF3_TIMESYNCEN   (1<<2)
#define MAX2769_CONF3_DATASYNCEN   (1<<1)
#define MAX2769_CONF3_STRMRST      (1<<0)
/* Some of the reserved bits are 1. */
#define MAX2769_CONF3_RESERVED     (1<<14)

/* MAX2769 PLL Configuration. */

#define MAX2769_PLLCONF_VCOEN             (1<<27)
#define MAX2769_PLLCONF_IVCO_LOW          (1<<26)
#define MAX2769_PLLCONF_IVCO_NORMAL       (0<<26)
/* PLLCONF bit 25 reserved. */
#define MAX2769_PLLCONF_REFOUTEN          (1<<24)
/* PLLCONF bit 23 reserved. */

#define MAX2769_PLLCONF_REFDIV_2X         (0<<21)
#define MAX2769_PLLCONF_REFDIV_DIV_4      (1<<21)
#define MAX2769_PLLCONF_REFDIV_DIV_2      (2<<21)
#define MAX2769_PLLCONF_REFDIV_DIV_NONE   (3<<21)

#define MAX2769_PLLCONF_IXTAL_OSC_NORMAL  (0<<19)
#define MAX2769_PLLCONF_IXTAL_BUFF_NORMAL (1<<19)
#define MAX2769_PLLCONF_IXTAL_OSC_MED     (2<<19)
#define MAX2769_PLLCONF_IXTAL_OSC_HIGH    (3<<19)

#define MAX2769_PLLCONF_XTALCAP(n)        ((n & 0x1F)<<14)
#define MAX2769_PLLCONF_LDMUX(n)          ((n & 0xF)<<10)

#define MAX2769_PLLCONF_ICP_1MA           (1<<9)
#define MAX2769_PLLCONF_ICP_0_5MA         (0<<9)

#define MAX2769_PLLCONF_PFDEN             (1<<8)
/* PLLCONF bit 7 reserved. */
#define MAX2769_PLLCONF_CPTEST(n)         ((n & 0x7)<<4)
#define MAX2769_PLLCONF_INTPLL            (1<<3)
#define MAX2769_PLLCONF_PWRSAV            (1<<2)
/* PLLCONF bits 1:0 reserved. */

/* Some of the reserved bits are 1. */
#define MAX2769_PLLCONF_RESERVED          (1<<23)

/* MAX2769 PLL Integer Division Ratio. */

#define MAX2769_DIV_NDIV(n) ((n & 0x7FFF)<<13)
#define MAX2769_DIV_RDIV(n) ((n & 0x3FF)<<3)
/* DIV bits 2:0 reserved. */

/* MAX2769 PLL Division Ratio. */

#define MAX2769_FDIV_FDIV(n)  ((n & 0xFFFFF)<<8)
/* DIV bits 7:0 reserved. */
/* Some of the reserved bits are 1. */
#define MAX2769_FDIV_RESERVED 0x70

/* MAX2769 DSP Interface. */

#define MAX2769_STRM_FRAMECOUNT(n) (n)

/* MAX2769 Clock Fractional Division Ratio. */

#define MAX2769_CLK_L_CNT(n) ((n & 0xFFF)<<16)
#define MAX2769_CLK_M_CNT(n) ((n & 0xFFF)<<4)
#define MAX2769_CLK_FCLKIN   (1<<3)
#define MAX2769_CLK_ADCCLK   (1<<2)
#define MAX2769_CLK_SERCLK   (1<<1)
#define MAX2769_CLK_MODE     (1<<0)

/** \} */

void max2769_write(u8 addr, u32 data);
void max2769_setup();

#endif
