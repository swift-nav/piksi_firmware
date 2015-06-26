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

#include "board/nap/track_channel.h"
#include "nmea.h"
#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"

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

/** Output NMEA sentence to all USARTs configured in NMEA mode.
 * The message is also sent to all dispatchers registered with
 * ::nmea_dispatcher_register.
 * \param s The NMEA sentence to output.
 */
void nmea_output(char *s)
{
  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  if ((ftdi_usart.mode == NMEA) && usart_claim(&ftdi_state, NMEA_MODULE)) {
    usart_write_dma(&ftdi_state.tx, (u8 *)s, strlen(s));
    usart_release(&ftdi_state);
  }

  if ((uarta_usart.mode == NMEA) && usart_claim(&uarta_state, NMEA_MODULE)) {
    usart_write_dma(&uarta_state.tx, (u8 *)s, strlen(s));
    usart_release(&uarta_state);
  }

  if ((uartb_usart.mode == NMEA) && usart_claim(&uartb_state, NMEA_MODULE)) {
    usart_write_dma(&uartb_state.tx, (u8 *)s, strlen(s));
    usart_release(&uartb_state);
  }

  __asm__("CPSIE i;");  /* Re-enable interrupts. */

  for (struct nmea_dispatcher *d = nmea_dispatchers_head; d; d = d->next)
    d->send(s);
}

void nmea_setup(void)
{

  SETTING("nmea", "gpgsv_msg_rate", gpgsv_msg_rate, TYPE_INT);
  SETTING("nmea", "gprmc_msg_rate", gprmc_msg_rate, TYPE_INT);
  SETTING("nmea", "gpvtg_msg_rate", gpvtg_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgll_msg_rate", gpgll_msg_rate, TYPE_INT);
}

/** Calculate the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or an `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s An NMEA sentence for which to generate the checksum.
 * \return The checksum value.
 */
u8 nmea_checksum(char *s)
{
  u8 sum = 0;

  if (*s == '$')
    s++;

  while (*s != '*' && *s) {
    sum ^= *s;
    s++;
  }

  return sum;
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

  unix_t = gps2time(*gps_t);
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

  char buf[80];
  u8 n = sprintf(buf,
                 "$GPGGA,%02d%02d%06.3f,"
                 "%02d%010.7f,%c,%03d%010.7f,%c,"
                 "%01d,%02d,%.1f,%.2f,M,,M,,",
                 t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                 lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                 fix_type, n_used, hdop, pos_llh[2]
                 );

  u8 sum = nmea_checksum(buf);

  sprintf(buf + n, "*%02X\r\n", sum);

  nmea_output(buf);
}

/** Assemble a NMEA GPGSA message and send it out NMEA USARTs.
 * NMEA GPGSA message contains DOP and active satellites.
 *
 * \param chans Pointer to tracking_channel_t struct.
 * \param dops  Pointer to dops_t struct.
 */
void nmea_gpgsa(const tracking_channel_t *chans, const dops_t *dops)
{
  char buf[80] = "$GPGSA,A,3,";
  char *bufp = buf + strlen(buf);

  for (u8 i = 0; i < 12; i++) {
    if (i < nap_track_n_channels && chans[i].state == TRACKING_RUNNING)
      bufp += sprintf(bufp, "%02d,", chans[i].prn + 1);
    else
      *bufp++ = ',';
  }

  if (dops)
    bufp += sprintf(bufp, "%.1f,%.1f,%.1f", dops->pdop, dops->hdop, dops->vdop);
  else
    bufp += sprintf(bufp, ",,");

  u8 sum = nmea_checksum(buf);

  sprintf(bufp, "*%02X\r\n", sum);

  nmea_output(buf);
}

/** Assemble a NMEA GPGSV message and send it out NMEA USARTs.
 * NMEA GPGSV message contains GPS satellites in view.
 *
 * \param n_used   Number of satellites currently being tracked.
 * \param nav_meas Pointer to navigation_measurement struct.
 * \param soln     Pointer to gnss_solution struct.
 */
void nmea_gpgsv(u8 n_used, const navigation_measurement_t *nav_meas,
                const gnss_solution *soln)
{
  if (n_used == 0)
    return;

  u8 n_mess = (n_used + 3) / 4;

  char buf[80];
  char *buf0 = buf + sprintf(buf, "$GPGSV,%d,", n_mess);
  char *bufp = buf0;

  u8 n = 0;
  double az, el;

  for (u8 i = 0; i < n_mess; i++) {
    bufp = buf0;
    bufp += sprintf(bufp, "%d,%d", i + 1, n_used);

    for (u8 j = 0; j < 4; j++) {
      if (n < n_used) {
        wgsecef2azel(nav_meas[n].sat_pos, soln->pos_ecef, &az, &el);
        bufp += sprintf(
          bufp, ",%02d,%02d,%03d,%02d",
          nav_meas[n].prn + 1,
          (u8)round(el * R2D),
          (u16)round(az * R2D),
          (u8)round(nav_meas[n].snr)
          );
      } else {
        bufp += sprintf(bufp, ",,,,");
      }
      n++;
    }

    sprintf(bufp, "*%02X\r\n", nmea_checksum(buf));
    nmea_output(buf);
  }

}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains minimum GPS data 
 *
 * \param nav_meas Pointer to navigation_measurement struct.
 * \param soln Pointer to gnss_solution struct
 * \param gps_t Pointer to the current GPS Time
 */
void nmea_gprmc(const navigation_measurement_t *nav_meas,
                const gnss_solution *soln, const gps_time_t *gps_t)
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

  unix_t = gps2time(*gps_t);
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

  double az, el;
  wgsecef2azel(nav_meas[0].sat_pos, soln->pos_ecef, &az, &el);

  char buf[100];
  u8 n = sprintf(buf,
                "$GPRMC,%02d%02d%06.3f,A," /* Command, Time (UTC), Valid */
                "%02d%010.7f,%c,%03d%010.7f,%c," /* Lat/Lon */
                "%06.2f,%05.1f," /* Speed, Course */
                "%02d%02d%02d," /* Date Stamp */
                ",", /* Variation */
                t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                velocity, course * R2D, 
                t.tm_mday, t.tm_mon, t.tm_year-100);

  u8 sum = nmea_checksum(buf);
  sprintf(buf + n, "*%02X\r\n", sum);
  nmea_output(buf);
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains course and speed
 *
 * \param nav_meas Pointer to navigation_measurement struct.
 * \param soln Pointer to gnss_solution struct
 */
void nmea_gpvtg(const navigation_measurement_t *nav_meas,
                const gnss_solution *soln)
{
  /* NMEA Parameters for GPVTG
   * Ex.
   * $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K
   *    |     |   |    |  |   |   |   |   |
   * Command  |  'T'   | 'M'  |  'N'  |  'K'
   *     True Course   |  Speed (K)   |
   *               Mag. course     Speed (km/hr)
   */

  double az, el;
  wgsecef2azel(nav_meas[0].sat_pos, soln->pos_ecef, &az, &el);

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

  char buf[80];
  u8 n = sprintf(buf, 
                  "$GPVTG,%05.1f,T," /* Command, course, */
                  ",M," /* Magnetic Course (omitted) */
                  "%06.2f,N,%06.2f,K", /* Speed (knots, km/hr) */
                  course* R2D,
                  vknots, vkmhr);
  
  u8 sum = nmea_checksum(buf);
  sprintf(buf + n, "*%02X\r\n", sum);
  nmea_output(buf);
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

  unix_t = gps2time(*gps_t);
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

  char buf[80];
  u8 n = sprintf(buf,
                "$GPGLL,"
                "%02d%010.7f,%c,%03d%010.7f,%c," /* Lat/Lon */
                "%02d%02d%06.3f,A", /* Time (UTC), Valid */
                lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                t.tm_hour, t.tm_min, t.tm_sec + frac_s);
  u8 sum = nmea_checksum(buf);
  sprintf(buf + n, "*%02X\r\n", sum);
  nmea_output(buf);
}
void nmea_send_msgs(gnss_solution *soln, u8 n, 
                    navigation_measurement_t *nm)
{
  if (gpgsv_msg_rate < 1) {
    gpgsv_msg_rate = 1;
  }
  if (gprmc_msg_rate < 1) {
    gpgsv_msg_rate = 1;
  }
  if (gpvtg_msg_rate < 1) {
    gpgsv_msg_rate = 1;
  }
  if (gpgll_msg_rate < 1) {
    gpgsv_msg_rate = 1;
  }
  DO_EVERY(gpgsv_msg_rate,
    nmea_gpgsv(n, nm, soln);
  );
  DO_EVERY(gprmc_msg_rate,
    nmea_gprmc(nm, soln, &soln->time);
  );
  DO_EVERY(gpvtg_msg_rate,
    nmea_gpvtg(nm, soln);
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
