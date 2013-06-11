/*
 * Copyright (C) 2013 Fergus Noble <fergusnoble@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <libswiftnav/coord_system.h>

#include "board/nap/track_channel.h"
#include "nmea.h"
#include "sbp.h"
#include "settings.h"
#include "peripherals/usart.h"

/** Output NMEA sentence to all USARTs configured in NMEA mode.
 * \param s The NMEA sentence to output.
 */
void nmea_output(char* s)
{
  /* Global interrupt disable to avoid concurrency/reentrancy problems. */
  __asm__("CPSID i;");

  if (settings.ftdi_usart.mode == NMEA)
    usart_write_dma(&ftdi_tx_state, (u8*)s, strlen(s));

  if (settings.uarta_usart.mode == NMEA)
    usart_write_dma(&uarta_tx_state, (u8*)s, strlen(s));

  if (settings.uartb_usart.mode == NMEA)
    usart_write_dma(&uartb_tx_state, (u8*)s, strlen(s));

  __asm__("CPSIE i;"); /* Re-enable interrupts. */
}

/** Calculate the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or an `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s An NMEA sentence for which to generate the checksum.
 * \return The checksum value.
 */
u8 nmea_checksum(char* s)
{
  u8 sum = 0;

  if (*s == '$')
    s++;

  while(*s != '*' && *s) {
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
void nmea_gpgga(gnss_solution* soln, dops_t* dops)
{
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(soln->time);
  gmtime_r(&unix_t, &t);

  double frac_s = fmod(soln->time.tow, 1.0);

  s8 lat_deg = (s8)((180.0/M_PI)*soln->pos_llh[0]);
  double lat_min = fabs(60*((180.0/M_PI)*soln->pos_llh[0] - lat_deg));
  s8 lon_deg = (s8)((180.0/M_PI)*soln->pos_llh[1]);
  double lon_min = fabs(60*((180.0/M_PI)*soln->pos_llh[1] - lon_deg));
  lat_deg = abs(lat_deg);
  lon_deg = abs(lon_deg);

  char lat_dir = soln->pos_llh[0] < 0 ? 'S' : 'N';
  char lon_dir = soln->pos_llh[1] < 0 ? 'W' : 'E';

  u8 fix_type = 1;

  char buf[80];
  u8 n = sprintf(buf,
    "$GPGGA,%02d%02d%06.3f,"
    "%02d%010.7f,%c,%03d%010.7f,%c,"
    "%01d,%02d,%.1f,%1.f,M,,M,,",
    t.tm_hour, t.tm_min, t.tm_sec + frac_s,
    lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
    fix_type, soln->n_used, dops->hdop, soln->pos_llh[2]
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
void nmea_gpgsa(tracking_channel_t* chans, dops_t* dops)
{
  char buf[80] = "$GPGSA,A,3,";
  char* bufp = buf + strlen(buf);

  for (u8 i=0; i<12; i++) {
    if (i < nap_track_n_channels && chans[i].state == TRACKING_RUNNING) {
      bufp += sprintf(bufp, "%02d,", chans[i].prn+1);
    } else {
      *bufp++ = ',';
    }
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
void nmea_gpgsv(u8 n_used, navigation_measurement_t* nav_meas,
                gnss_solution* soln)
{
  if (n_used == 0)
    return;

  u8 n_mess = (n_used + 3) / 4;

  char buf[80];
  char* buf0 = buf + sprintf(buf, "$GPGSV,%d,", n_mess);
  char* bufp = buf0;

  u8 n = 0;
  double az, el;

  for (u8 i=0; i<n_mess; i++) {
    bufp = buf0;
    bufp += sprintf(bufp, "%d,%d", i+1, n_used);
    for (u8 j=0; j<4; j++) {
      if (n < n_used) {
        wgsecef2azel(nav_meas[n].sat_pos, soln->pos_ecef, &az, &el);
        bufp += sprintf(
          bufp, ",%02d,%02d,%03d,%02d",
          nav_meas[n].prn+1,
          (u8)round(el*180.0/M_PI),
          (u16)round(az*180.0/M_PI),
          (u8)(10.0 * nav_meas[n].snr)
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

