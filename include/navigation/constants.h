/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
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

#ifndef INCLUDE_NAVIGATION_CONSTANTS_H_
#define INCLUDE_NAVIGATION_CONSTANTS_H_

namespace bfs {
/* Semi-major axis, WGS-84 defined m */
static constexpr double SEMI_MAJOR_AXIS_LENGTH_M = 6378137.0;
/* Flattening, WGS-84 defined */
static constexpr double FLATTENING = 1.0 / 298.257223563;
/* Semi-minor axis, m (derived) */
static constexpr double SEMI_MINOR_AXIS_LENGTH_M = SEMI_MAJOR_AXIS_LENGTH_M *
                                                   (1.0 - FLATTENING);
/* First eccentricity, squared (derived) */
static constexpr double E2 = 2.0 * FLATTENING - FLATTENING * FLATTENING;
/* Used for Olson's method */
static constexpr double A1 = SEMI_MAJOR_AXIS_LENGTH_M * E2;
static constexpr double A2 = A1 * A1;
static constexpr double A3 = A1 * E2 / 2.0;
static constexpr double A4 = 2.5 * A2;
static constexpr double A5 = A1 + A3;
static constexpr double A6 = 1.0 - E2;
}  // namespace bfs

#endif  // INCLUDE_NAVIGATION_CONSTANTS_H_
