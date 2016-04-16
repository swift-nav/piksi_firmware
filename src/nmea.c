/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/coord_system.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "nmea.h"
#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"
#include "main.h"

const char NMEA_MODULE[] = "nmea";

u32 gpgsv_msg_rate = 10;
u32 gprmc_msg_rate = 10;
u32 gpvtg_msg_rate = 10;
u32 gpgll_msg_rate = 10;
static struct nmea_dispatcher *nmea_dispatchers_head;
/** \addtogroup io
 * \{ */

/** \defgroup nmea NMEA
 * Send messages in NMEA format.
 * \{ */

#define NMEA_SUFFIX_LEN 6  /* How much room to leave for the NMEA
                              checksum, CRLF + null termination,
                              i.e. "*%02X\r\n\0" */

/** Some helper macros for functions generating NMEA sentences. */

/** NMEA_SENTENCE_START: declare a buffer and set up some pointers
 * max_len = max possible length of the body of the message
 * (not including suffix)
 */
#define NMEA_SENTENCE_START(max_len) char sentence_buf[max_len + NMEA_SUFFIX_LEN]; \
  char *sentence_bufp = sentence_buf; \
  char * const sentence_buf_end = sentence_buf + max_len;

/** NMEA_SENTENCE_PRINTF: use like printf, can use multiple times
    within a sentence. */
#define NMEA_SENTENCE_PRINTF(fmt, ...) do { \
    sentence_bufp += snprintf(sentence_bufp, sentence_buf_end - sentence_bufp, fmt, ##__VA_ARGS__); \
    if (sentence_bufp >= sentence_buf_end) \
      sentence_bufp = sentence_buf_end; } while (0)

/** NMEA_SENTENCE_DONE: append checksum and dispatch. */
#define NMEA_SENTENCE_DONE() do { \
    if (sentence_bufp == sentence_buf_end) \
      log_warn("NMEA %.6s cut off", sentence_buf); \
    nmea_append_checksum(sentence_buf, sizeof(sentence_buf)); \
    nmea_output(sentence_buf, sentence_bufp - sentence_buf + NMEA_SUFFIX_LEN); \
  } while (0)

/** Output NMEA sentence to all USARTs configured in NMEA mode.
 * The message is also sent to all dispatchers registered with
 * ::nmea_dispatcher_register.
 * \param s The NMEA sentence to output.
 */
static void nmea_output(char *s, size_t size)
{
  static MUTEX_DECL(send_mutex);
  chMtxLock(&send_mutex);

  if ((ftdi_usart.mode == NMEA) && usart_claim(&ftdi_state, NMEA_MODULE)) {
    usart_write(&ftdi_state, (u8 *)s, size);
    usart_release(&ftdi_state);
  }

  if ((uarta_usart.mode == NMEA) && usart_claim(&uarta_state, NMEA_MODULE)) {
    usart_write(&uarta_state, (u8 *)s, size);
    usart_release(&uarta_state);
  }

  if ((uartb_usart.mode == NMEA) && usart_claim(&uartb_state, NMEA_MODULE)) {
    usart_write(&uartb_state, (u8 *)s, size);
    usart_release(&uartb_state);
  }

  chMtxUnlock(&send_mutex);

  for (struct nmea_dispatcher *d = nmea_dispatchers_head; d; d = d->next)
    d->send(s, size);
}

void nmea_setup(void)
{

  SETTING("nmea", "gpgsv_msg_rate", gpgsv_msg_rate, TYPE_INT);
  SETTING("nmea", "gprmc_msg_rate", gprmc_msg_rate, TYPE_INT);
  SETTING("nmea", "gpvtg_msg_rate", gpvtg_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgll_msg_rate", gpgll_msg_rate, TYPE_INT);
}

/** Calculate and append the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or a `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s A null-terminated NMEA sentence, up to and optionally
 * including the '*'
 *
 * \param size Length of the buffer.
 *
 */
static void nmea_append_checksum(char *s, size_t size)
{
  u8 sum = 0;
  char *p = s;

  /* '$' header not included in checksum calculation */
  if (*p == '$')
    p++;

  /* '*'  not included in checksum calculation */
  while (*p != '*' &&
         *p &&
         p + NMEA_SUFFIX_LEN < s + size) {
    sum ^= *p;
    p++;
  }

  sprintf(p, "*%02X\r\n", sum);
}

/** Assemble a NMEA GPGGA message and send it out NMEA USARTs.
 * NMEA GPGGA message contains Global Positioning System Fix Data.
 *
 * \param soln Pointer to gnss_solution struct.
 * \param dops Pointer to dops_t struct.
 */
void nmea_gpgga(const double pos_llh[3], const gps_time_t *gps_t, u8 n_used,
                u8 fix_type, double hdop)
{
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);

  double frac_s = fmod(gps_t->tow, 1.0);

  s16 lat_deg = R2D * (pos_llh[0]);
  double lat_min = MINUTES(pos_llh[0]);
  s16 lon_deg = R2D * (pos_llh[1]);
  double lon_min = MINUTES(pos_llh[1]);
  lat_deg = abs(lat_deg);
  lon_deg = abs(lon_deg);

  char lat_dir = pos_llh[0] < 0 ? 'S' : 'N';
  char lon_dir = pos_llh[1] < 0 ? 'W' : 'E';

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGGA,%02d%02d%06.3f,"
                       "%02d%010.7f,%c,%03d%010.7f,%c,"
                       "%01d,%02d,%.1f,%.2f,M,,M,,",
                       t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                       lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                       fix_type, n_used, hdop, pos_llh[2]
                       );
  NMEA_SENTENCE_DONE();
}

/** Assemble a NMEA GPGSA message and send it out NMEA USARTs.
 * NMEA GPGSA message contains DOP and active satellites.
 *
 * \param prns      Array of prns to output
 * \param num_prns  Length of prns array
 * \param dops      Pointer to dops_t struct.
 */
void nmea_gpgsa(const u8 *prns, u8 num_prns, const dops_t *dops)
{
  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGSA,A,3,");

  for (u8 i = 0; i < 12; i++) {
    if (i < num_prns)
      NMEA_SENTENCE_PRINTF("%02d,", prns[i]);
    else
      NMEA_SENTENCE_PRINTF(",");
  }

  if (dops)
    NMEA_SENTENCE_PRINTF("%.1f,%.1f,%.1f", dops->pdop, dops->hdop, dops->vdop);
  else
    NMEA_SENTENCE_PRINTF(",,");

  NMEA_SENTENCE_DONE();
}

/** Assemble a NMEA GPGSV message and send it out NMEA USARTs.
 * NMEA GPGSV message contains GPS satellites in view.
 *
 * \param n_used   Number of satellites currently being tracked.
 * \param nav_meas Array of navigation_measurement structs.
 * \param soln     Pointer to gnss_solution struct.
 */
void nmea_gpgsv(u8 n_used, const navigation_measurement_t *nav_meas,
                const gnss_solution *soln)
{
  if (n_used == 0)
    return;

  u8 n_mess = (n_used + 3) / 4;


  u8 n = 0;
  double az, el;

  for (u8 i = 0; i < n_mess; i++) {
    NMEA_SENTENCE_START(120);
    NMEA_SENTENCE_PRINTF("$GPGSV,%d,%d,%d", n_mess, i+1, n_used);

    for (u8 j = 0; j < 4; j++) {
      if (n < n_used) {
        wgsecef2azel(nav_meas[n].sat_pos, soln->pos_ecef, &az, &el);
        /* TODO: only include GPS signals */
        NMEA_SENTENCE_PRINTF(",%02d,%02d,%03d,%02d",
          nav_meas[n].sid.sat,
          (u8)round(el * R2D),
          (u16)round(az * R2D),
          (u8)round(nav_meas[n].snr)
          );
      } else {
        NMEA_SENTENCE_PRINTF(",,,,");
      }
      n++;
    }

    NMEA_SENTENCE_DONE();
  }

}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains minimum GPS data
 *
 * \param soln Pointer to gnss_solution struct
 * \param gps_t Pointer to the current GPS Time
 */
void nmea_gprmc(const gnss_solution *soln, const gps_time_t *gps_t)
{

  /* NMEA Parameters
   * Ex.
   * $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
   *   |      |    |    |    |    |     |   |      |      |     |  |  |
   * Command  |    |   Lat  N/S   |     |   |      | Date Stamp | W/E |
   *    Time (UTC) |            Long   W/E  |  True Course      |   Cksum
   *            Validity (A-OK)           Speed            Variation
   * Variation is ignored as we have no way to maintain that information
   * currently
   */
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);
  double frac_s = fmod(gps_t->tow, 1.0);

  s16 lat_deg = R2D * (soln->pos_llh[0]);
  double lat_min = MINUTES(soln->pos_llh[0]);
  s16 lon_deg = R2D * (soln->pos_llh[1]);
  double lon_min = MINUTES(soln->pos_llh[1]);
  lat_deg = abs(lat_deg);
  lon_deg = abs(lon_deg);

  char lat_dir = soln->pos_llh[0] < 0 ? 'S' : 'N';
  char lon_dir = soln->pos_llh[1] < 0 ? 'W' : 'E';

  float velocity;
  float x,y,z;
  x = soln->vel_ned[0];
  y = soln->vel_ned[1];
  z = soln->vel_ned[2];
  float course = atan2(y,x);

  /* Conversion to magnitue knots */
  velocity = MS2KNOTTS(x,y,z);

  NMEA_SENTENCE_START(140);
  NMEA_SENTENCE_PRINTF(
                "$GPRMC,%02d%02d%06.3f,A," /* Command, Time (UTC), Valid */
                "%02d%010.7f,%c,%03d%010.7f,%c," /* Lat/Lon */
                "%06.2f,%05.1f," /* Speed, Course */
                "%02d%02d%02d," /* Date Stamp */
                ",", /* Variation */
                t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                velocity, course * R2D,
                t.tm_mday, t.tm_mon + 1, t.tm_year % 100);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains course and speed
 *
 * \param soln Pointer to gnss_solution struct
 */
void nmea_gpvtg(const gnss_solution *soln)
{
  /* NMEA Parameters for GPVTG
   * Ex.
   * $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K
   *    |     |   |    |  |   |   |   |   |
   * Command  |  'T'   | 'M'  |  'N'  |  'K'
   *     True Course   |  Speed (K)   |
   *               Mag. course     Speed (km/hr)
   */

  float vknots, vkmhr;
  float x,y,z;
  x = soln->vel_ned[0];
  y = soln->vel_ned[1];
  z = soln->vel_ned[2];
  float course = atan2(y,x);

  /* Conversion to magnitue knots */
  vknots = MS2KNOTTS(x,y,z);
  /* Conversion to magnitue km/hr */
  vkmhr = MS2KMHR(x,y,z);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF(
                  "$GPVTG,%05.1f,T," /* Command, course, */
                  ",M," /* Magnetic Course (omitted) */
                  "%06.2f,N,%06.2f,K", /* Speed (knots, km/hr) */
                  course* R2D,
                  vknots, vkmhr);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPGLL message and send it out NMEA USARTs.
 * NMEA GLL contains course and speed
 *
 * \param soln Pointer to gnss_solution struct
 * \param gpt_t Pointer to the current GPS Time
 */
void nmea_gpgll(const gnss_solution *soln, const gps_time_t *gps_t)
{
  /* NMEA Parameters for GPGLL
   * Ex.
   * $GPGLL,5133.81,N,00042.25,W,225444,A*75
   *   |       |    |    |     |    |   |
   * Command   |   N/S Lon    E/W   | Valid
   *          LAT                  UTC
   */
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);

  double frac_s = fmod(gps_t->tow, 1.0);
  s16 lat_deg = R2D * (soln->pos_llh[0]);
  double lat_min = MINUTES(soln->pos_llh[0]);
  s16 lon_deg = R2D * (soln->pos_llh[1]);
  double lon_min =  MINUTES(soln->pos_llh[1]);
  lat_deg = abs(lat_deg);
  lon_deg = abs(lon_deg);

  char lat_dir = soln->pos_llh[0] < 0 ? 'S' : 'N';
  char lon_dir = soln->pos_llh[1] < 0 ? 'W' : 'E';

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGLL,"
                "%02d%010.7f,%c,%03d%010.7f,%c," /* Lat/Lon */
                "%02d%02d%06.3f,A", /* Time (UTC), Valid */
                lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                t.tm_hour, t.tm_min, t.tm_sec + frac_s);
  NMEA_SENTENCE_DONE();
}


/** Generate and send periodic NMEA GPGSV, GPRMC, GPVTG, GPGLL
 * (but not GPGGA) messages.
 *
 * Called from solution thread.
 *
 * \param soln     Pointer to gnss_solution struct.
 * \param n        Number of satellites in use
 * \param nav_meas Array of n navigation_measurement structs.
 */
void nmea_send_msgs(gnss_solution *soln, u8 n,
                    navigation_measurement_t *nm)
{
  DO_EVERY(gpgsv_msg_rate,
    nmea_gpgsv(n, nm, soln);
  );
  DO_EVERY(gprmc_msg_rate,
    nmea_gprmc(soln, &soln->time);
  );
  DO_EVERY(gpvtg_msg_rate,
    nmea_gpvtg(soln);
  );
  DO_EVERY(gpgll_msg_rate,
    nmea_gpgll(soln, &soln->time);
  );
}
/** \cond */
void _nmea_dispatcher_register(struct nmea_dispatcher *d)
{
  d->next = nmea_dispatchers_head;
  nmea_dispatchers_head = d;
}
/** \endcond */

/** \} */

/** \} */
