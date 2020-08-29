/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_UTILS_H_
#define INCLUDE_NAVIGATION_UTILS_H_

#include "Eigen/Core"

namespace navigation {
/* Rate of change of LLA given NED velocity */
Eigen::Vector3f LlaRate(const Eigen::Vector3f &ned_vel, const Eigen::Vector3d &lla);
/* Skew symmetric matrix from a given vector w */
template<typename T>
Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1> &w) {
  Eigen::Matrix<T, 3, 3> c;
  c(0, 0) =  0.0;  c(0, 1) = -w(2); c(0, 2) =  w(1);
  c(1, 0) =  w(2); c(1, 1) =  0.0;  c(1, 2) = -w(0);
  c(2, 0) = -w(1); c(2, 1) =  w(0); c(2, 2) =  0.0;
  return c;
}

}  // namespace navigation

#endif  // INCLUDE_NAVIGATION_UTILS_H_
