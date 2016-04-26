/*
 * Copyright (C) 2013-2016 Swift Navigation Inc.
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
#include <libswiftnav/time.h>

#include "board/nap/track_channel.h"
#include "track.h"
#include "nmea.h"
#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"
#include "main.h"

const char NMEA_MODULE[] = "nmea";

static u32 gpgsv_msg_rate = 10;
static u32 gprmc_msg_rate = 10;
static u32 gpvtg_msg_rate =  1;
static u32 gpgll_msg_rate = 10;
static u32 gpzda_msg_rate = 10;
static u32 gpgsa_msg_rate = 10;

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
#define NMEA_SENTENCE_START(max_len) \
  char sentence_buf[max_len + NMEA_SUFFIX_LEN]; \
  char *sentence_bufp = sentence_buf; \
  char * const sentence_buf_end = sentence_buf + max_len;

/** NMEA_SENTENCE_PRINTF: use like printf, can use multiple times
    within a sentence. */
#define NMEA_SENTENCE_PRINTF(fmt, ...) do { \
    sentence_bufp += snprintf(sentence_bufp, sentence_buf_end - sentence_bufp, fmt, ##__VA_ARGS__); \
    if (sentence_bufp >= sentence_buf_end) \
      sentence_bufp = sentence_buf_end; \
    } while (0)

/** NMEA_SENTENCE_DONE: append checksum and dispatch.
 * \note According to section 5.3.1 of the NMEA 0183 spec, sentences are
 *       terminated with <CR><LF>. The sentence_buf is null_terminated.
 *       The call to nmea_output has been modified to remove the NULL.
 *       This will also affect all registered dispatchers
 */
#define NMEA_SENTENCE_DONE() do { \
    if (sentence_bufp == sentence_buf_end) \
      log_warn("NMEA %.6s cut off", sentence_buf); \
    nmea_append_checksum(sentence_buf, sizeof(sentence_buf)); \
    nmea_output(sentence_buf, sentence_bufp - sentence_buf + NMEA_SUFFIX_LEN-1); \
  } while (0)

/** Output NMEA sentence to all USARTs configured in NMEA mode.
 * The message is also sent to all dispatchers registered with
 * ::nmea_dispatcher_register.

 * \param s The NMEA sentence to output.
 * \param size This is the C-string size, not including the null character
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
  SETTING("nmea", "gpzda_msg_rate", gpzda_msg_rate, TYPE_INT);
  SETTING("nmea", "gpgsa_msg_rate", gpgsa_msg_rate, TYPE_INT);
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
 * \param pos_llh       Array of Latitude [rad], Longitude [rad], Height [m].
 * \param gps_t         Pointer to GPS time struct (week and TOW).
 * \param n_used        Number of used satellites.
 * \param fix_type      GPS Quality indicator.
 * \param hdop          Horizontal DOP.
 * \param diff_age      Age of differential corrections [s].
 * \param station_id    Differential reference station ID.
 */
void nmea_gpgga(const double pos_llh[3], const gps_time_t *gps_t, u8 n_used,
                u8 fix_type, double hdop, double diff_age, u16 station_id)
{
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);

  double frac_s  = fmod(gps_t->tow, 1.0);

  double lat     = fabs(round(R2D * pos_llh[0] * 1e8) /1e8);
  double lon     = fabs(round(R2D * pos_llh[1] * 1e8) /1e8);

  char   lat_dir = pos_llh[0] < 0.0 ? 'S' : 'N';
  u16    lat_deg = (u16)lat;
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char   lon_dir = pos_llh[1] < 0.0 ? 'W' : 'E';
  u16    lon_deg = (u16)lon;
  double lon_min = (lon - (double)lon_deg) * 60.0;

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGGA,%02d%02d%06.3f,"
                       "%02u%010.7f,%c,%03u%010.7f,%c,"
                       "%01d,%02d,%.1f,%.2f,M,0.0,M,",
                       t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                       lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                       fix_type, n_used, hdop, pos_llh[2]
                       );
  if (fix_type > 1) {
    NMEA_SENTENCE_PRINTF("%.1f,%04d",diff_age, station_id & 0x3FF); /* ID range is 0000 to 1023 */
  } else {
    NMEA_SENTENCE_PRINTF(",");
  }

  NMEA_SENTENCE_DONE();
}

/** Assemble a NMEA GPGSA message and send it out NMEA USARTs.
 * NMEA GPGSA message contains GNSS DOP and Active Satellites.
 *
 * \param prns      Array of PRNs to output.
 * \param num_prns  Number of valid PRNs in array.
 * \param dops      Pointer to DOP struct (PDOP, HDOP, VDOP).
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
 * NMEA GPGSV message contains GNSS Satellites In View.
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
    NMEA_SENTENCE_PRINTF("$GPGSV,%u,%u,%02u", n_mess, i+1, n_used);

    for (u8 j = 0; j < 4; j++) {
      if (n < n_used) {
        wgsecef2azel(nav_meas[n].sat_pos, soln->pos_ecef, &az, &el);
        /* TODO: only include GPS signals */
        NMEA_SENTENCE_PRINTF(",%02u,%02u,%03u,%02u",
          nav_meas[n].sid.sat,
          (u8)round(el * R2D),
          (u16)round(az * R2D),
          (u8)round(nav_meas[n].snr)
          );
      }
      n++;
    }

    NMEA_SENTENCE_DONE();
  }

}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains Recommended Minimum Specific GNSS Data.
 *
 * \param soln Pointer to gnss_solution struct.
 * \param gps_t Pointer to the current GPS Time.
 */
void nmea_gprmc(const gnss_solution *soln, const gps_time_t *gps_t)
{
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);

  double frac_s  = fmod(gps_t->tow, 1.0);

  double lat     = fabs(round(R2D * soln->pos_llh[0] * 1e8) / 1e8);
  double lon     = fabs(round(R2D * soln->pos_llh[1] * 1e8) / 1e8);

  char   lat_dir = soln->pos_llh[0] < 0.0 ? 'S' : 'N';
  u16    lat_deg = (u16)lat;
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char   lon_dir = soln->pos_llh[1] < 0.0 ? 'W' : 'E';
  u16    lon_deg = (u16)lon;
  double lon_min = (lon - (double)lon_deg) * 60.0;

  float x,y,z;
  x = soln->vel_ned[0];
  y = soln->vel_ned[1];
  z = soln->vel_ned[2];
  float course = R2D * atan2(y,x);
  if (course < 0.0) {
    course += 360.0;
  }
  /* Conversion to magnitue knots */
  float vknots = MS2KNOTTS(x,y,z);

  NMEA_SENTENCE_START(140);
  NMEA_SENTENCE_PRINTF(
                "$GPRMC,%02d%02d%06.3f,A,"       /* Command, Time (UTC), Valid */
                "%02u%010.7f,%c,%03u%010.7f,%c," /* Lat/Lon */
                "%.2f,%05.1f,"                   /* Speed, Course */
                "%02d%02d%02d,"                  /* Date Stamp */
                ",",                             /* Magnetic Variation */
                t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                vknots, course,
                t.tm_mday, t.tm_mon + 1, t.tm_year % 100);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains Course Over Ground & Ground Speed.
 *
 * \param soln Pointer to gnss_solution struct.
 */
void nmea_gpvtg(const gnss_solution *soln)
{
  float x,y,z;
  x = soln->vel_ned[0];
  y = soln->vel_ned[1];
  z = soln->vel_ned[2];
  float course = R2D * atan2(y,x);
  if (course < 0.0) {
    course += 360.0;
  }
  /* Conversion to magnitue knots */
  float vknots = MS2KNOTTS(x,y,z);
  /* Conversion to magnitue km/hr */
  float vkmhr = MS2KMHR(x,y,z);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF(
                  "$GPVTG,%05.1f,T," /* Command, course, */
                  ",M,"              /* Magnetic Course (omitted) */
                  "%.2f,N,%.2f,K",   /* Speed (knots, km/hr) */
                  course,
                  vknots, vkmhr);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPGLL message and send it out NMEA USARTs.
 * NMEA GLL contains Geographic Position Latitude/Longitude.
 *
 * \param soln  Pointer to gnss_solution struct.
 * \param gpt_t Pointer to the current GPS Time.
 */
void nmea_gpgll(const gnss_solution *soln, const gps_time_t *gps_t)
{
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);

  double frac_s  = fmod(gps_t->tow, 1.0);

  double lat     = fabs(round(R2D * soln->pos_llh[0] * 1e8) / 1e8);
  double lon     = fabs(round(R2D * soln->pos_llh[1] * 1e8) / 1e8);

  char   lat_dir = soln->pos_llh[0] < 0.0 ? 'S' : 'N';
  u16    lat_deg = (u16)lat;
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char   lon_dir = soln->pos_llh[1] < 0.0 ? 'W' : 'E';
  u16    lon_deg = (u16)lon;
  double lon_min = (lon - (double)lon_deg) * 60.0;

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGLL,"
                "%02u%010.7f,%c,%03u%010.7f,%c," /* Lat/Lon */
                "%02d%02d%06.3f,A",              /* Time (UTC), Valid */
                lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                t.tm_hour, t.tm_min, t.tm_sec + frac_s);
  NMEA_SENTENCE_DONE();
}

/** Assemble an NMEA GPZDA message and send it out NMEA USARTs.
 * NMEA ZDA contains UTC Time and Date.
 *
 * \param gps_t Pointer to the current GPS Time.
 */
void nmea_gpzda(const gps_time_t *gps_t)
{
  time_t unix_t;
  struct tm t;

  unix_t = gps2time(gps_t);
  gmtime_r(&unix_t, &t);
  double frac_s = fmod(gps_t->tow, 1.0);

  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF(
                "$GPZDA,%02d%02d%06.3f," /* Command, Time (UTC) */
                "%02d,%02d,%d,"          /* Date Stamp */
                ",",                     /* Time zone */
                t.tm_hour, t.tm_min, t.tm_sec + frac_s,
                t.tm_mday, t.tm_mon + 1, 1900 + t.tm_year);
  NMEA_SENTENCE_DONE();

} // nmea_gpzda()


static void nmea_assemble_gpgsa(const dops_t *dops)
{
  /* Assemble list of currently tracked GPS PRNs */
  u8 prns[nap_track_n_channels];
  u8 num_prns = 0;
  for (u32 i=0; i<nap_track_n_channels; i++) {
    tracking_channel_lock(i);
    if (tracking_channel_running(i)) {
      gnss_signal_t sid = tracking_channel_sid_get(i);
      if (sid_to_constellation(sid) == CONSTELLATION_GPS) {
        prns[num_prns++] = sid.sat;
      }
    }
    tracking_channel_unlock(i);
  }
  /* Send GPGSA message */
  nmea_gpgsa(prns, num_prns, dops);
}



/** Generate and send periodic NMEA GPRMC, GPGLL, GPVTG, GPZDA, GPGSA and GPGSV.
 * (but not GPGGA) messages.
 *
 * Called from solution thread.
 *
 * \param soln          Pointer to gnss_solution struct.
 * \param n             Number of satellites in use.
 * \param nav_meas      Array of n navigation_measurement structs.
 * \param skip_velocity If TRUE then don't output any messages with velocity.
 */
void nmea_send_msgs(gnss_solution *soln, u8 n,
                    navigation_measurement_t *nm,
                    const dops_t *dops,
                    bool skip_velocity)
{
  if (!skip_velocity) {
    DO_EVERY(gprmc_msg_rate,
      nmea_gprmc(soln, &soln->time);
    );
  }
  DO_EVERY(gpgll_msg_rate,
    nmea_gpgll(soln, &soln->time);
  );
  if (!skip_velocity) {
    DO_EVERY(gpvtg_msg_rate,
      nmea_gpvtg(soln);
    );
  }
  DO_EVERY(gpzda_msg_rate,
    nmea_gpzda(&soln->time);
  );
  DO_EVERY(gpgsa_msg_rate,
    nmea_assemble_gpgsa(dops);
  );
  DO_EVERY(gpgsv_msg_rate,
    nmea_gpgsv(n, nm, soln);
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
