
#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>

#include "track.h"

void nmea_gpgga(gnss_solution* soln, dops_t* dops);
void nmea_gpgsa(tracking_channel_t* chans, dops_t* dops);
void nmea_gpgsv(u8 n_used, navigation_measurement_t* nav_meas,
                gnss_solution* soln);

