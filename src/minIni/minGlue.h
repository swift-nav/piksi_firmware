/*  Glue functions for the minIni library, based on the C/C++ stdio library
 *
 *  Or better said: this file contains macros that maps the function interface
 *  used by minIni to the standard C/C++ file I/O functions.
 *
 *  By CompuPhase, 2008-2012
 *  This "glue file" is in the public domain. It is distributed without
 *  warranties or conditions of any kind, either express or implied.
 */

/* map required file I/O types and functions to the cfs filesystem */
#include "cfs/cfs.h"

#include <stdio.h>

#define INI_READONLY
#define INI_FILETYPE                  int
#define ini_openread(filename,file)   ((*(file) = cfs_open((filename),CFS_READ)) != -1)
#define ini_close(file)               (cfs_close(*(file)))
/* minIni requires fgets(3) like read, but cfs only provides a read(2). */
int ini_read(char *buffer, int size, int *fd);

