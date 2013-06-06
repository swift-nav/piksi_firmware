/*
 * Copyright (C) 2011 Fergus Noble <fergusnoble@gmail.com>
 * Copyright (C) 2013 Colin Beighley <colinbeighley@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SWIFTNAV_NAP_CONF_H
#define SWIFTNAV_NAP_CONF_H

#include <libopencm3/cm3/common.h>

/* Configuration flash addresses of interest. */
#define FLASH_NAP_PARAMS_ADDR 0xD0000
#define FLASH_NAP_GIT_HASH_ADDR 0xE0000
#define FLASH_NAP_GIT_UNCLEAN_ADDR (FLASH_NAP_GIT_HASH_ADDR + 20)

void get_nap_parameters();

void get_nap_git_hash(u8 git_hash[]);
u8 get_nap_git_unclean();

#endif /* SWIFTNAV_NAP_CONF_H */
