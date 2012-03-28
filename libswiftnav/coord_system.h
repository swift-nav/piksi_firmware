/*
 * Copyright (c) 2008, Scott Gleason
 * Copyright (c) 2010, C.O. Lee Boyce, Jr.
 * Copyright (c) 2010, Fergus Noble
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the authors' names nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SWIFTLIB_COORD_SYSTEM_H
#define SWIFTLIB_COORD_SYSTEM_H

void wgsllh2ecef(const double *llh, double *ecef);
void wgsecef2llh(const double const ecef[3], double llh[3]);

/*
 * Be careful in choosing which of these two functions you need, wgsecef2ned_rt
 * references from the ref pos altitude, i.e. translates the position as well
 * as rotating. This is _not_ what you want e.g. for velocities which should
 * only be rotated using the wgsecef2ned_r.
 */
void wgsecef2ned_rt(const double ecef[3], const double ref_ecef[3], double NED[3]);
void wgsecef2ned_r(const double ecef[3], const double ref_ecef[3], double NED[3]);
void wgsecef2azel(const double ecef[3], const double ref_ecef[3],
                  double* azimuth, double* elevation);

#endif

