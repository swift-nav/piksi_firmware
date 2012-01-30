// Gives u8, s16 etc

#ifndef SWIFTNAV_INT_TYPES_H
#define SWIFTNAV_INT_TYPES_H

#define USING_LIBOPENCM3
#ifdef USING_LIBOPENCM3
  #include <libopencm3/cm3/common.h>
#else
  #include <stdint.h>
  typedef uint8_t u8;
  typedef uint16_t u16;
  typedef int8_t s8;
  typedef int16_t s16;
  typedef int32_t s32;

#endif

#endif
