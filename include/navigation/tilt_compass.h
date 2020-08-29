/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_TILT_COMPASS_H_
#define INCLUDE_NAVIGATION_TILT_COMPASS_H_

#include "Eigen/Core"
#include "Eigen/Dense"

namespace navigation {
/*
* Yaw, pitch, and roll from 3-axis accelerometer and 
* 3-axis magnetometer measurements
*/
Eigen::Vector3f TiltCompass(const Eigen::Vector3f &accel, const Eigen::Vector3f &mag);

}  // namespace navigation

#endif  // INCLUDE_NAVIGATION_TILT_COMPASS_H_
