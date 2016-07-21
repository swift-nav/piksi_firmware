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

#ifndef SRC_NDB_INTERNAL_H_
#define SRC_NDB_INTERNAL_H_

#define MAX_NDB_FILE_VERSION_LEN 64
extern u8 ndb_file_version[MAX_NDB_FILE_VERSION_LEN];

/** NDB file handler */
typedef int ndb_file_handle_t;
/* Information element size */
typedef u16 ndb_ie_size_t;
/* Information element index in the array */
typedef u8 ndb_ie_index_t;

/** NDB File */
typedef struct __attribute__((packed)) {
  const char* name; /**< Name of the file */
  ndb_file_handle_t fh; /**< File handle */
  const u32 expected_size; /**< Expected file size */
  const u16 data_size; /** Size of data element */
  const u16 n_elements; /** Number of data elements */
} ndb_file_t;

/* Maximum waiting time for write request, milliseconds */
#define NV_WRITE_REQ_TIMEOUT 100

#define NDB_IE_DIRTY (1 << 0) /**< IE needs to be written to NVM */
#define NDB_IE_VALID (1 << 1) /**< value in RAM has been set */
#define NDB_MD_DIRTY (1 << 2) /**< Metadata needs to be written to NVM */
#define NDB_ENQUEUED (1 << 3) /**<  */
#define NDB_IE_IS_VALID(md_ptr) ((md_ptr)->nv_data.state & NDB_IE_VALID)

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
  ndb_ie_index_t index;
  ndb_file_t *file;
  struct ndb_element_metadata *next;
} ndb_element_metadata_t;

extern u8 ndb_file_end_mark;

void ndb_init();
void ndb_start();
void ndb_lock();
void ndb_unlock();

ndb_timestamp_t ndb_get_timestamp();
void ndb_load_data(ndb_file_t *f,
                   const char* ftype,
                   void *data,
                   ndb_element_metadata_t* metadata,
                   const size_t el_size,
                   const size_t el_number);
enum ndb_op_code ndb_update(void* new, enum ndb_data_source src,
                            ndb_element_metadata_t *md);
void ndb_retrieve(void* out, void* cached, size_t size);
enum ndb_op_code ndb_write_file_data(ndb_file_t *f, cfs_offset_t o, void *b,
                                     size_t l);

#endif
