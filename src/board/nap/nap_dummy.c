
#include <string.h>

#include "board/nap/track_channel.h"

u8 nap_track_n_channels;
u8 nap_acq_fft_index_bits;
u8 nap_acq_downsample_stages;

void nap_setup(void)
{
}

void nap_callbacks_setup(void)
{
}

void nap_track_init_wr_blocking(u8 channel, u8 prn, s32 carrier_phase,
                                u16 code_phase)
{
  (void)channel;
  (void)prn;
  (void)carrier_phase;
  (void)code_phase;
}

void nap_track_code_wr_blocking(u8 channel, gnss_signal_t sid)
{
  (void)channel;
  (void)sid;
}

void nap_track_update_wr_blocking(u8 channel, s32 carrier_freq,
                                  u32 code_phase_rate, u8 rollover_count,
                                  u8 corr_spacing)
{
  (void)channel;
  (void)carrier_freq;
  (void)code_phase_rate;
  (void)rollover_count;
  (void)corr_spacing;
}

u64 nap_timing_count(void)
{
  return 0;
}

void nap_acq_code_wr_blocking(gnss_signal_t sid)
{
  (void)sid;
}

void nap_acq_load_wr_enable_blocking(void)
{
}

void nap_acq_init_wr_params_blocking(s16 carrier_freq)
{
  (void)carrier_freq;
}

u32 nap_error_rd_blocking(void)
{
  return 0;
}

void nap_timing_strobe(u32 falling_edge_count)
{
  (void)falling_edge_count;
}

bool nap_timing_strobe_wait(u32 timeout)
{
  (void)timeout;
  return true;
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
  strcpy(version_string, "version");
  return strlen(version_string);
}

u32 nap_conf_rd_version(void)
{
  return 0;
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

