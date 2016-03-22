/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_NDB_H_
#define SRC_NDB_H_

#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/signal.h>
#include <ch.h>
#include <cfs/cfs-coffee.h>
#include <cfs/cfs.h>
#include "signal.h"

enum ndb_op_code
{
  NDB_ERR_NONE = 0, /**< No error */
  NDB_ERR_MISSING_IE, /**< DB doesn't contain value of this IE */
  NDB_ERR_UNSUPPORTED,
  NDB_ERR_FILE_IO,
  NDB_ERR_INIT_DONE,
  NDB_ERR_BAD_PARAM,
  NDB_ERR_TIMEOUT
};

enum ndb_data_source
{
  NDB_DS_UNDEFINED = 0,
  NDB_DS_INIT,
  NDB_DS_RECEIVER,
  NDB_DS_SBP
};

#define MAX_NDB_FILE_VERSION_LEN 64
#define ndb_file_version GIT_VERSION

/** NDB IE update counter type */
typedef u8 ndb_update_counter_t;
/** NDB file handler */
typedef int ndb_file_handle_t;
/** NDB Timestamp */
typedef systime_t ndb_timestamp_t;
/* Information element size */
typedef u8 ndb_ie_size_t;
/* Information element index in the array */
typedef u8 ndb_ie_index_t;

/** NDB File */
typedef struct
{
  char* name; /**< Name of the file */
  ndb_file_handle_t fh; /**< File handle */
  u32 expected_size; /**< Expected file size */
} ndb_file_t;

typedef union
{
  ephemeris_t ephe;
  almanac_t alma;
  u32 l2c_capabilities;
} ndb_element_t;

/* Maximum waiting time for write request, milliseconds */
#define NV_WRITE_REQ_TIMEOUT 100

#define NDB_IE_DIRTY (1 << 0) /**< needs to be written to NVM */
#define NDB_IE_VALID (1 << 1) /**< value in RAM has been set */
#define NDB_IE_IS_VALID(md_ptr) (md_ptr->nv_data.state & NDB_IE_VALID)

typedef struct __attribute__((packed)) ndb_element_metadata_nv
{
  ndb_timestamp_t received_at;
  enum ndb_data_source source;
  u8 state;
} ndb_element_metadata_nv_t;

typedef struct __attribute__((packed)) ndb_element_metadata
{
  ndb_element_metadata_nv_t nv_data;
  ndb_update_counter_t update_c;
  void *data;
  ndb_ie_size_t data_size;
  ndb_ie_index_t index;
  u16 n_elements;
  ndb_file_t *file;
  struct ndb_element_metadata *next;
} ndb_element_metadata_t;

#define NDB_WEAK __attribute__ ((weak, alias ("ndb_not_implemented")))
enum ndb_op_code ndb_not_implemented() __attribute__ ((weak));
inline enum ndb_op_code ndb_not_implemented()
{
  return NDB_ERR_UNSUPPORTED;
}

void ndb_lock(u8 lock);
ndb_timestamp_t ndb_get_timestamp();
enum ndb_op_code ndb_not_implemented();
enum ndb_op_code ndb_init();
enum ndb_op_code ndb_load_data(ndb_file_t *f, void *b,
                               ndb_element_metadata_t* b_md, size_t el_size,
                               size_t n);

enum ndb_op_code ndb_update(void* cached, void* new, size_t size,
                            enum ndb_data_source src,
                            ndb_element_metadata_t *md,
                            u8 and_state_mask, u8 or_state_mask);
void ndb_retrieve(void* out, void* cached, size_t size);
enum ndb_op_code ndb_write_file_data(ndb_file_t *f, cfs_offset_t o, void *b,
                                     size_t l);

enum ndb_op_code ndb_invalidate(ndb_element_metadata_t *md);

enum ndb_op_code ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e);
enum ndb_op_code ndb_ephemeris_store(ephemeris_t *e, enum ndb_data_source);
enum ndb_op_code ndb_ephemeris_info(gnss_signal_t sid, u8* v, u8* h,
                                    gps_time_t* toe, u8* fit_interval);
enum ndb_op_code ndb_update_cache_ephemeris(ephemeris_t *cached_e,
                                            ndb_update_counter_t *uc);

enum ndb_op_code ndb_almanac_read(gnss_signal_t sid, almanac_t *a);
enum ndb_op_code ndb_almanac_store(almanac_t *a, enum ndb_data_source);
enum ndb_op_code ndb_update_cache_almanac(almanac_t *cached_a,
                                          ndb_update_counter_t *uc);

/**
 * L2CM specific
 */
enum ndb_op_code ndb_gps_l2cm_l2c_cap_read(u32 *l2c_cap);
enum ndb_op_code ndb_gps_l2cm_l2c_cap_store(u32 *l2c_cap,
                                            enum ndb_data_source src);

#endif /* SRC_NDB_H_ */
