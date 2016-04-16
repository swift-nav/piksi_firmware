
#include <string.h>

#include "board/nap/track_channel.h"

u8 nap_acq_fft_index_bits;
u8 nap_acq_downsample_stages;

void nap_callbacks_setup(void)
{
}

u32 nap_error_rd_blocking(void)
{
  return 0;
}

u32 nap_rw_ext_event(u8 *event_pin, ext_event_trigger_t *event_trig,
		     ext_event_trigger_t next_trig)
{
  (void)event_pin;
  (void)event_trig;
  (void)next_trig;
  return 0;
}

u8 nap_conf_rd_version_string(char version_string[])
{
  strcpy(version_string, "v0.16");
  return strlen(version_string);
}

s32 nap_conf_rd_serial_number(void)
{
  return 0;
}

const char * nap_conf_rd_hw_rev_string(void)
{
  return "hw version";
}

void nap_pps(u64 rising_edge_count_8x)
{
  (void)rising_edge_count_8x;
}

void nap_pps_width(u32 falling_edge_count)
{
  (void)falling_edge_count;
}

bool nap_timing_strobe_wait(u32 timeout)
{
  (void)timeout;
  return true;
}

