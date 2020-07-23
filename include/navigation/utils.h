/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_UTILS_H_
#define INCLUDE_NAVIGATION_UTILS_H_

#include "navigation/constants.h"
#include "Eigen/Core"

namespace navigation {

Eigen::Vector3f LlaRate(const Eigen::Vector3f &ned_vel, const Eigen::Vector3d &lla) {
  Eigen::Vector3f lla_dot;
  double lat = lla(0);
  double alt = lla(2);
  double denom = abs(1 - (constants::E2 * sin(lat) * sin(lat)));
  double sqrt_denom = denom;
  double Rns = constants::SEMI_MAJOR_AXIS_LENGTH_M * (1 - constants::E2) / (denom * sqrt_denom); 
  double Rew = constants::SEMI_MAJOR_AXIS_LENGTH_M / sqrt_denom;
  lla_dot(0) = ned_vel(0) / (Rns + alt); 
  lla_dot(1) = ned_vel(1) / ((Rew + alt) * cos(lat)); 
  lla_dot(2) = -ned_vel(2);
  return lla_dot;
}
/* Skew symmetric matrix from a given vector w */
template<typename T>
Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1> &w) {
  Eigen::Matrix<T, 3, 3> c;
  c(0,0) =  0.0;	c(0,1) = -w(2);	c(0,2) =  w(1);
  c(1,0) =  w(2);	c(1,1) =  0.0;	c(1,2) = -w(0);
  c(2,0) = -w(1);	c(2,1) =  w(0);	c(2,2) =  0.0;
  return c;
}

}  // namespace navigation


#endif  // INCLUDE_NAVIGATION_UTILS_H_
