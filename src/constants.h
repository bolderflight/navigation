/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef NAVIGATION_SRC_CONSTANTS_H_ // NOLINT
#define NAVIGATION_SRC_CONSTANTS_H_

namespace bfs {
/* Semi-major axis, WGS-84 defined, m */
static constexpr double SEMI_MAJOR_AXIS_LENGTH_M = 6378137.0;
/* Flattening */
static constexpr double FLATTENING = 1.0 / 298.257223563;
/* Semi-minor axis, m (derived) */
static constexpr double SEMI_MINOR_AXIS_LENGTH_M = 6356752.3142;
/* First eccentricity (derived) */
static constexpr double ECC = 8.1819190842622e-2;
/* First eccentricity, squared (derived) */
static constexpr double ECC2 = 6.69437999014e-3;
/* Angular velocity of the Earth, rad/s */
static constexpr double WE_RADPS = 7292115.0e-11;
/* Angular velocity of the Earth according to ICD-GPS-200, rad/s */
static constexpr double WE_GPS_RADPS = 7292115.1467e-11;
/* Earth's Gravitational Constant, m^3/s^2 */
static constexpr double GM_M3PS2 = 3986004.418e8;
/* Earth's Gravitational Constant according to ICD-GPS-200, m^3/s^2 */
static constexpr double GM_GPS_M3PS2 = 3986005.0e8;

}  // namespace bfs

#endif  // NAVIGATION_SRC_CONSTANTS_H_ NOLINT
