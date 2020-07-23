/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_CONSTANTS_H_
#define INCLUDE_NAVIGATION_CONSTANTS_H_

namespace navigation {
namespace constants {
/* Semi-major axis, WGS-84 defined m */
static constexpr double SEMI_MAJOR_AXIS_LENGTH_M = 6378137.0;
/* Flattening, WGS-84 defined */
static constexpr double FLATTENING = 1.0 / 298.257223563;
/* Semi-minor axis, m (derived) */
static constexpr double SEMI_MINOR_AXIS_LENGTH_M = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - FLATTENING);
/* First eccentricity, squared (derived) */
static constexpr double E2 = 2.0 * FLATTENING - FLATTENING * FLATTENING;
/* Used for Olson's method */
static constexpr double A1 = SEMI_MAJOR_AXIS_LENGTH_M * E2;
static constexpr double A2 = A1 * A1;
static constexpr double A3 = A1 * E2 / 2.0;
static constexpr double A4 = 2.5 * A2;
static constexpr double A5 = A1 + A3;
static constexpr double A6 = 1.0 - E2;
}  // namespace constants
}  // namespace navigation

#endif  // INCLUDE_NAVIGATION_CONSTANTS_H_
