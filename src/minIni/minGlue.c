#include "minIni/minGlue.h"

int ini_read(char *buffer, int size, int *fd)
{
	int i;
	for (i = 0; i < size; i++, buffer++) {
		if (cfs_read(*fd, buffer, 1) == 0)
			break;
		if (*buffer == '\n')
			break;
	}
	return i;
}

