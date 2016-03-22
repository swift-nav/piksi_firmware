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
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <libswiftnav/logging.h>
#include "libsbp/piksi.h"
#include "sbp.h"
#include "ndb.h"

typedef Thread *ndb_thread_t;

static ndb_thread_t ndb_thread;
#define NDB_THREAD_PRIORITY (LOWPRIO)
static WORKING_AREA_CCM(ndb_thread_wa, 1756);
static msg_t ndb_service_thread(void*);
MUTEX_DECL(data_access);

ndb_element_metadata_t *wq_first = NULL;
ndb_element_metadata_t *wq_last = NULL;
CONDVAR_DECL(wq_new_data);
MUTEX_DECL(wq_access);

static enum ndb_op_code ndb_wq_put(ndb_element_metadata_t* md);
static enum ndb_op_code ndb_wq_get(ndb_element_metadata_t** md);

static enum ndb_op_code ndb_open_file(ndb_file_t *file, char *v);
static enum ndb_op_code ndb_read(ndb_file_t *f, void *first_element, size_t s);

static enum ndb_op_code do_nv_writes(bool *data_write_ok, bool *md_write_ok);
static void do_sbp_updates();

enum ndb_op_code ndb_p1_init();
enum ndb_op_code ndb_p3_init() NDB_WEAK;

void ndb_p1_sbp_update();
void ndb_p3_sbp_update() NDB_WEAK;

enum ndb_op_code ndb_gps_l2cm_l2c_cap_read(u32 *l2c_cap) NDB_WEAK;
enum ndb_op_code ndb_gps_l2cm_l2c_cap_store(u32 *l2c_cap,
                                            enum ndb_data_source src) NDB_WEAK;

enum ndb_op_code ndb_wq_put(ndb_element_metadata_t* md)
{
  chMtxLock(&wq_access);
  if (NULL == wq_last) {
    wq_first = wq_last = md;
  } else {
    wq_last->next = md;
    wq_last = md;
  }
  md->next = NULL;
  chCondSignal(&wq_new_data);
  chMtxUnlock();
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_wq_get(ndb_element_metadata_t** md)
{
  chMtxLock(&wq_access);
  if (NULL == wq_first) {
    /* NOTE: chCondwaitTimeout does not care of
     locking mutex again when timeout occurs*/
    if (chCondWaitTimeout(&wq_new_data,
                          NV_WRITE_REQ_TIMEOUT) == RDY_TIMEOUT) {
      *md = NULL;
      return NDB_ERR_TIMEOUT;
    }
  }

  *md = wq_first;
  wq_first = wq_first->next;

  if (NULL == wq_first) {
    wq_last = NULL;
  }

  chMtxUnlock();
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_init()
{
  ndb_p1_init();
  ndb_p3_init();

  ndb_thread = chThdCreateStatic(ndb_thread_wa, sizeof(ndb_thread_wa),
                                 NDB_THREAD_PRIORITY,
                                 ndb_service_thread,
                                 NULL);
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_load_data(ndb_file_t *f, void *b,
                               ndb_element_metadata_t* b_md, size_t el_size,
                               size_t n)
{
  size_t ds = el_size * n;
  size_t mds = sizeof(ndb_element_metadata_nv_t) * n;
  ndb_element_metadata_nv_t md_nv[n];

  if (ndb_open_file(f, ndb_file_version) == NDB_ERR_NONE) {
    if (cfs_seek(f->fh, 0, CFS_SEEK_SET) != 0) {
      cfs_close(f->fh);
      f->fh = -1;
      return NDB_ERR_FILE_IO;
    }
    if (ndb_read(f, b, ds) == NDB_ERR_NONE) {
      if (ndb_read(f, md_nv, mds) == NDB_ERR_NONE) {
        u32 i;
        for (i = 0; i < n; i++) {
          b_md[i].nv_data = md_nv[i];
        }
        return NDB_ERR_NONE;
      }
    }

    memset(b, 0, ds);
    memset(md_nv, 0, mds);
    memset(b_md, 0, sizeof(ndb_element_metadata_t) * n);

    if (ndb_write_file_data(f, 0, b, ds) == NDB_ERR_NONE) {
      if (ndb_write_file_data(f, ds, md_nv, mds) == NDB_ERR_NONE) {
        return NDB_ERR_INIT_DONE;
      }
    }
  }

  return NDB_ERR_FILE_IO;
}

/**
 * Returns time stamp in seconds.
 */
ndb_timestamp_t ndb_get_timestamp()
{
  return chTimeNow();
}

static msg_t ndb_service_thread(void* p)
{
  (void) (p);
  chRegSetThreadName("ndb");
  bool data_write_ok = false;
  bool md_write_ok = false;

  while (true) {
    if (NDB_ERR_TIMEOUT == do_nv_writes(&data_write_ok, &md_write_ok)) {
      do_sbp_updates();
    } else {
      if (data_write_ok && md_write_ok)
        log_info("Data and metadata were written to the NDB file");
      else {
        if (!data_write_ok)
          log_error("Error writing data to the NDB file");
        if (!md_write_ok)
          log_error("Error writing metadata to the NDB file");
      }
    }
  }
  return 0;
}

enum ndb_op_code do_nv_writes(bool *data_write_ok, bool *md_write_ok)
{
  ndb_element_metadata_t* md = NULL;
  enum ndb_op_code ret;
  int changed;

  if (NDB_ERR_TIMEOUT == ndb_wq_get(&md)) {
    return NDB_ERR_TIMEOUT;
  } else {
    ndb_element_metadata_t md_copy;
    ndb_element_t buf;
    ndb_lock(1);
    memcpy(&buf, md->data, md->data_size);
    memcpy(&md_copy, md, sizeof(ndb_element_metadata_t));
    ndb_lock(0);
    ret = ndb_write_file_data(md->file, md->data_size * md->index, &buf,
                              md->data_size);
    *data_write_ok = NDB_ERR_NONE == ret;
    if (data_write_ok) {
      ndb_lock(1);
      changed = memcmp(md->data, &buf, md->data_size);
      ndb_lock(0);
      if (!changed) {
        /*
         * Data hasn't changed while i/o operation was running -
         * mark as saved and write metadata to NVM.
         * */
        md->nv_data.state &= ~NDB_IE_DIRTY;
        cfs_offset_t offset = md->data_size * md->n_elements
            + sizeof(ndb_element_metadata_nv_t) * md->index;
        ret = ndb_write_file_data(md->file, offset, &md_copy,
                                  sizeof(ndb_element_metadata_nv_t));
        *md_write_ok = NDB_ERR_NONE == ret;
      } else {
        /* Data has changed - schedule saving once again */
        md->nv_data.state |= NDB_IE_DIRTY;
        ndb_wq_put(md);
      }
    }
  }

  return NDB_ERR_NONE;
}

void do_sbp_updates()
{
  ndb_p1_sbp_update();
  ndb_p3_sbp_update();
}

/**
 * Opens NDB file that stores information elements of certain type.
 * This function checks if version of the file matches the passed one
 * and if it doesn't creates empty file automatically.
 */
enum ndb_op_code ndb_open_file(ndb_file_t *file, char *version)
{
  char ver[MAX_NDB_FILE_VERSION_LEN];

  file->fh = cfs_open(file->name, CFS_READ + CFS_WRITE);
  if (file->fh < 0) {
    cfs_coffee_reserve(file->name, file->expected_size);
    cfs_coffee_configure_log(file->name, 256 * 2, 256);
    file->fh = cfs_open(file->name, CFS_READ + CFS_WRITE);
    if (file->fh < 0) {
      return NDB_ERR_FILE_IO;
    }
  }

  if (cfs_read(file->fh, ver, sizeof(ndb_file_version))
      == sizeof(ndb_file_version)) {
    if (memcmp(&ver, version, sizeof(ndb_file_version)) == 0) {
      return NDB_ERR_NONE;
    } else {
      cfs_close(file->fh);
      cfs_remove(file->name);
      return ndb_open_file(file, version);
    }
  }

  if (cfs_seek(file->fh, 0, CFS_SEEK_SET) != 0) {
    cfs_close(file->fh);
    file->fh = -1;
    return NDB_ERR_FILE_IO;
  }

  if (cfs_write(file->fh, version, sizeof(ndb_file_version))
      != sizeof(ndb_file_version)) {
    cfs_close(file->fh);
    file->fh = -1;
    return NDB_ERR_FILE_IO;
  }

  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_write_file_data(ndb_file_t *f, cfs_offset_t o, void *b,
                                     size_t l)
{
  int offset = sizeof(ndb_file_version) + o;
  int ret = cfs_seek(f->fh, offset, CFS_SEEK_SET);

  if (ret == offset) {
    if (cfs_write(f->fh, b, l) == (int) l) {
      return NDB_ERR_NONE;
    }
  }
  return NDB_ERR_FILE_IO;
}

/**
 * Reading from NDB file.
 * This function is to be called only during initialization of NDB.
 * It is intended to read all information elements collection from
 * file into memory.
 */
enum ndb_op_code ndb_read(ndb_file_t *file, void *first_element, size_t size)
{
  if (file->fh < 0) {
    return NDB_ERR_FILE_IO;
  }

  if (cfs_seek(file->fh, sizeof(ndb_file_version), CFS_SEEK_SET)
      == (cfs_offset_t) -1) {
    cfs_close(file->fh);
    file->fh = -1;
    return NDB_ERR_FILE_IO;
  }
  return
      cfs_read(file->fh, first_element, size) == (int) size ?
          NDB_ERR_NONE : NDB_ERR_FILE_IO;
}

void ndb_lock(u8 lock)
{
  lock ? chMtxLock(&data_access) : chMtxUnlock();
}

void ndb_retrieve(void* out, void* cached, size_t size)
{
  ndb_lock(1);
  memcpy(out, cached, size);
  ndb_lock(0);
}

enum ndb_op_code ndb_update(void* cached, void* new, size_t size,
                            enum ndb_data_source src,
                            ndb_element_metadata_t *md, u8 and_state_mask,
                            u8 or_state_mask)
{
  ndb_lock(1);
  if (memcmp(new, cached, size) == 0) {
    ndb_lock(0);
    return NDB_ERR_NONE;
  }
  memcpy(cached, new, size);
  md->update_c++;
  if (!md->update_c)
    md->update_c++; /* should never be 0 */
  md->nv_data.state &= and_state_mask;
  md->nv_data.state |= or_state_mask;
  md->nv_data.received_at = ndb_get_timestamp();
  md->nv_data.state |= NDB_IE_DIRTY;
  md->nv_data.source = src;

  ndb_wq_put(md);

  ndb_lock(0);
  return NDB_ERR_NONE;
}
