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

#include <libswiftnav/logging.h>

#include "../peripherals/spi_wrapper.h"
#include "../settings.h"
#include "frontend.h"

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
#define MAX2769_CONF1_LNAMODE_MASK  (~(3<<13))

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

/** \addtogroup board
 * \{ */

/** \defgroup max MAX2769
 * Interface to configure the MAX2769 RF Front-end.
 * \{ */

/** Write to one of the MAX2769 registers.
 *
 * \param addr MAX2769 register to write to.
 * \param data Data to write to register (28 bits).
 */
static void max2769_write(u8 addr, u32 data)
{
  u32 write_word = ((data << 4) & 0xFFFFFFF0) | (addr & 0x0F);

  spi_slave_select(SPI_SLAVE_FRONTEND);
  spi_slave_xfer(SPI_SLAVE_FRONTEND, (write_word >> 24) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FRONTEND, (write_word >> 16) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FRONTEND, (write_word >>  8) & 0xFF);
  spi_slave_xfer(SPI_SLAVE_FRONTEND, (write_word >>  0) & 0xFF);
  spi_slave_deselect(SPI_SLAVE_FRONTEND);
}

static u32 max2769_conf1;
static u32 max2769_conf2;
static u32 max2769_conf3;
static u32 max2769_pllconf;
static u32 max2769_div;
static u32 max2769_fdiv;
static u32 max2769_clk;

/** Setup MAX2769 GPIOs and write default settings to MAX2769 registers. */
void frontend_configure(void)
{
  max2769_conf1 = MAX2769_CONF1_CHIPEN |
                  MAX2769_CONF1_ILNA1(15) |
                  MAX2769_CONF1_ILNA2(3) |
                  MAX2769_CONF1_ILO(3) |
                  MAX2769_CONF1_IMIX(3) |
                  MAX2769_CONF1_MIXPOLE_13MHZ |
                  MAX2769_CONF1_MIXEN |
                  MAX2769_CONF1_ANTEN |      /* Supply DC bias to external antenna */
                  //MAX2769_CONF1_FCEN(7) |  /* 1 MHz IF filter center freq. */
                  //MAX2769_CONF1_FCEN(43) | /* 2 Mhz IF filter center freq. */
                  MAX2769_CONF1_FCEN(21) |   /* 4 Mhz IF filter center freq. */
                  //MAX2769_CONF1_FCEN(42) | /* 8 Mhz IF filter center freq. */
                  MAX2769_CONF1_FBW_2_5MHZ |
                  MAX2769_CONF1_F3OR5_5 |
                  MAX2769_CONF1_FCENX_BP |
                  MAX2769_CONF1_FGAIN_HIGH |
                  MAX2769_CONF1_LNAMODE_GATED;
  max2769_write(MAX2769_CONF1, max2769_conf1);

  max2769_conf2 = //MAX2769_CONF2_IQEN |
                  MAX2769_CONF2_GAINREF(170) |  /* optimal for 2 bits */
                  //MAX2769_CONF2_GAINREF(82) | /* optimal for 3 bits */
                  MAX2769_CONF2_AGCMODE_INDEP |
                  MAX2769_CONF2_FORMAT_SIGN_MAG |
                  MAX2769_CONF2_BITS_1 |
                  MAX2769_CONF2_DRVCFG_CMOS |
                  MAX2769_CONF2_LOEN;
  max2769_write(MAX2769_CONF2, max2769_conf2);

  max2769_conf3 = MAX2769_CONF3_RESERVED |
                  MAX2769_CONF3_GAININ(58) |
                  MAX2769_CONF3_FSLOWEN |
                  MAX2769_CONF3_ADCEN |
                  MAX2769_CONF3_DRVEN |
                  MAX2769_CONF3_FOFSTEN |
                  MAX2769_CONF3_FILTEN |
                  MAX2769_CONF3_FHIPEN |
                  MAX2769_CONF3_PGAIEN |
                  //MAX2769_CONF3_PGAQEN |
                  /* STRM stuff was set before but its unused,
                   * can leave as zeros. */
                  0;
  max2769_write(MAX2769_CONF3, max2769_conf3);

  max2769_pllconf = MAX2769_PLLCONF_RESERVED |
                  MAX2769_PLLCONF_VCOEN |
                  MAX2769_PLLCONF_REFOUTEN |
                  //MAX2769_PLLCONF_REFDIV_DIV_2 |  /* 8.184 MHz sample rate */
                  MAX2769_PLLCONF_REFDIV_DIV_NONE | /* 16.368 MHz sample rate */
                  MAX2769_PLLCONF_IXTAL_BUFF_NORMAL |
                  MAX2769_PLLCONF_XTALCAP(0b10000) |
                  MAX2769_PLLCONF_LDMUX(0) |
                  MAX2769_PLLCONF_ICP_1MA |
                  MAX2769_PLLCONF_CPTEST(0) |
                  MAX2769_PLLCONF_INTPLL;
  max2769_write(MAX2769_PLLCONF, max2769_pllconf);

  max2769_div = //MAX2769_DIV_NDIV(1538) | /* 2 * 1.023 MHz IF */
                MAX2769_DIV_NDIV(1536) |  /* 4 * 1.023 MHz IF */
                MAX2769_DIV_RDIV(16);
  max2769_write(MAX2769_DIV, max2769_div);

  max2769_fdiv = MAX2769_FDIV_RESERVED |
                 MAX2769_FDIV_FDIV(0x80000);
  max2769_write(MAX2769_FDIV, max2769_fdiv);

  max2769_clk = MAX2769_CLK_L_CNT(1) |
                MAX2769_CLK_M_CNT(4095) |
                MAX2769_CLK_SERCLK;
  max2769_write(MAX2769_CLK, max2769_clk);

}

antenna_type_t antenna = EXTERNAL;

static bool antenna_changed(struct setting *s, const char *val)
{
  bool ant_status;
  ant_status = frontend_ant_status();
  if (s->type->from_string(s->type->priv, s->addr, s->len, val))
  {
    max2769_conf1 &= MAX2769_CONF1_LNAMODE_MASK & ~MAX2769_CONF1_ANTEN;

    switch (antenna) {
    default:
    case AUTO:
      max2769_conf1 |= MAX2769_CONF1_LNAMODE_GATED | MAX2769_CONF1_ANTEN;
      break;
    case PATCH:
      max2769_conf1 |= MAX2769_CONF1_LNAMODE_LNA1 | MAX2769_CONF1_ANTEN;
      if (ant_status) {
        log_warn("Patch antenna selected, but an external "
                 "antenna appears to be connected."); 
      }
      break;
    case EXTERNAL:
      max2769_conf1 |= MAX2769_CONF1_LNAMODE_LNA2 | MAX2769_CONF1_ANTEN;
      if (!ant_status) { 
        log_warn("External antenna selected, but no external " 
                 "antenna appears to be connected."); 
      }
      break;
    case EXTERNAL_AC:
      max2769_conf1 |= MAX2769_CONF1_LNAMODE_LNA2;
      break;
    }
    max2769_write(MAX2769_CONF1, max2769_conf1);
    log_info("Antenna changed to: %s", val);

    return true;
  }
  return false;
}

/* Setup user settings hooks etc. */
void frontend_setup(void)
{
  static const char const *antenna_enum[] = {"Auto", "Patch", "External", "External (no bias)", NULL};
  static struct setting_type antenna_setting;
  int TYPE_ANTENNA = settings_type_register_enum(antenna_enum, &antenna_setting);
  SETTING_NOTIFY("frontend", "antenna_selection", antenna, TYPE_ANTENNA, antenna_changed);
}

bool frontend_ant_status(void)
{
  return palReadLine(LINE_MAX_ANT_FLAG);
}

antenna_type_t frontend_ant_setting(void)
{
  return antenna;
}
/** \} */

/** \} */

