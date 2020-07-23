/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_TILT_COMPASS_H_
#define INCLUDE_NAVIGATION_TILT_COMPASS_H_

#include "types/types.h"
#include "Eigen/Core"
#include "Eigen/Dense"

namespace navigation {

Eigen::Vector3f TiltCompass(const Eigen::Vector3f &accel, const Eigen::Vector3f &mag) {
  Eigen::Vector3f ypr;
  Eigen::Vector3f a = accel;
  Eigen::Vector3f m = mag;
  /* Normalize accel and mag */
  a.normalize();
  m.normalize();
  /* Pitch */
  ypr(1) = asinf(a(0));
  /* Roll */
  ypr(2) = asinf(-a(1) / cosf(ypr(1)));
  /* Yaw */
  ypr(0) = atan2f(m(2) * sinf(ypr(2)) - m(1) * cosf(ypr(2)), m(0) * cosf(ypr(1)) + m(1) * sinf(ypr(1)) * sinf(ypr(2)) + m(2) * sinf(ypr(1)) * cosf(ypr(2)));
  return ypr;
}

}  // namespace navigation

#endif  // INCLUDE_NAVIGATION_TILT_COMPASS_H_
