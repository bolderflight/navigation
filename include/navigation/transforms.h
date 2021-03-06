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

#ifndef INCLUDE_NAVIGATION_TRANSFORMS_H_
#define INCLUDE_NAVIGATION_TRANSFORMS_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "units/units.h"

namespace bfs {
/* Euler (3-2-1) to Direction Cosine Matrix (DCM) */
template<typename T>
Eigen::Matrix<T, 3, 3> angle2dcm(const Eigen::Matrix<T, 3, 1> &eul) {
  Eigen::Matrix<T, 3, 3> dcm;
  T psi = eul(0, 0);
  T theta = eul(1, 0);
  T phi = eul(2, 0);
  T cos_theta = std::cos(theta);
  T sin_theta = std::sin(theta);
  T cos_phi = std::cos(phi);
  T sin_phi = std::sin(phi);
  T cos_psi = std::cos(psi);
  T sin_psi = std::sin(psi);
  dcm(0, 0) = cos_theta * cos_psi;
  dcm(1, 0) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
  dcm(2, 0) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
  dcm(0, 1) = cos_theta * sin_psi;
  dcm(1, 1) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
  dcm(2, 1) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
  dcm(0, 2) = -sin_theta;
  dcm(1, 2) = sin_phi * cos_theta;
  dcm(2, 2) = cos_phi * cos_theta;
  return dcm;
}
/* Direction Cosine Matrix (DCM) to Euler (3-2-1) */
template<typename T>
Eigen::Matrix<T, 3, 1> dcm2angle(const Eigen::Matrix<T, 3, 3> &dcm) {
  Eigen::Matrix<T, 3, 1> euler;
  euler(0, 0) = std::atan2(dcm(0, 1), dcm(0, 0));
  euler(1, 0) = -std::asin(dcm(0, 2));
  euler(2, 0) = std::atan2(dcm(1, 2), dcm(2, 2));
  return euler;
}
/* Euler (3-2-1) to quaternion */
template<typename T>
Eigen::Quaternion<T> angle2quat(const Eigen::Matrix<T, 3, 1> &euler) {
  T psi = euler(0, 0) / static_cast<T>(2);
  T the = euler(1, 0) / static_cast<T>(2);
  T phi = euler(2, 0) / static_cast<T>(2);
  Eigen::Quaternion<T> q;
  q.w() = std::cos(psi) * std::cos(the) * std::cos(phi) + std::sin(psi) *
          std::sin(the) * std::sin(phi);
  q.x() = std::cos(psi) * std::cos(the) * std::sin(phi) - std::sin(psi) *
          std::sin(the) * std::cos(phi);
  q.y() = std::cos(psi) * std::sin(the) * std::cos(phi) + std::sin(psi) *
          std::cos(the) * std::sin(phi);
  q.z() = std::sin(psi) * std::cos(the) * std::cos(phi) - std::cos(psi) *
          std::sin(the) * std::sin(phi);
  return q;
}
/* Quaternion to Euler (3-2-1) */
template<typename T>
Eigen::Matrix<T, 3, 1> quat2angle(const Eigen::Quaternion<T> &q) {
  T m11 = static_cast<T>(2) * q.w() * q.w() + static_cast<T>(2) * q.x() *
          q.x() - static_cast<T>(1);
  T m12 = static_cast<T>(2) * q.x() * q.y() + static_cast<T>(2) * q.w() *
          q.z();
  T m13 = static_cast<T>(2) * q.x() * q.z() - static_cast<T>(2) * q.w() *
          q.y();
  T m23 = static_cast<T>(2) * q.y() * q.z() + static_cast<T>(2) * q.w() *
          q.x();
  T m33 = static_cast<T>(2) * q.w() * q.w() + static_cast<T>(2) * q.z() *
          q.z() - static_cast<T>(1);
  Eigen::Matrix<T, 3, 1> euler;
  euler(0, 0) = atan2(m12, m11);
  euler(1, 0) = asin(-m13);
  euler(2, 0) = atan2(m23, m33);
  return euler;
}
/* DCM to Quaternion */
template<typename T>
Eigen::Quaternion<T> dcm2quat(const Eigen::Matrix<T, 3, 3> &dcm) {
  Eigen::Quaternion<T> q;
  q.w() = static_cast<T>(0.5) * std::sqrt(static_cast<T>(1) + dcm(0, 0) +
          dcm(1, 1) + dcm(2, 2));
  q.x() = (dcm(1, 2) - dcm(2, 1)) / (static_cast<T>(4) * q.w());
  q.y() = (dcm(2, 0) - dcm(0, 2)) / (static_cast<T>(4) * q.w());
  q.z() = (dcm(0, 1) - dcm(1, 0)) / (static_cast<T>(4) * q.w());
  return q;
}
/* Quaternion to DCM */
template<typename T>
Eigen::Matrix<T, 3, 3> quat2dcm(const Eigen::Quaternion<T> &q) {
  Eigen::Matrix<T, 3, 3> dcm;
  dcm(0, 0) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) +
              static_cast<T>(2) * q.x() * q.x();
  dcm(1, 0) = static_cast<T>(2) * q.x() * q.y() - static_cast<T>(2) *
              q.w() * q.z();
  dcm(2, 0) = static_cast<T>(2) * q.x() * q.z() + static_cast<T>(2) *
              q.w() * q.y();
  dcm(0, 1) = static_cast<T>(2) * q.x() * q.y() + static_cast<T>(2) *
              q.w() * q.z();
  dcm(1, 1) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) +
              static_cast<T>(2) * q.y() * q.y();
  dcm(2, 1) = static_cast<T>(2) * q.y() * q.z() - static_cast<T>(2) *
              q.w() * q.x();
  dcm(0, 2) = static_cast<T>(2) * q.x() * q.z() - static_cast<T>(2) *
              q.w() * q.y();
  dcm(1, 2) = static_cast<T>(2) * q.y() * q.z() + static_cast<T>(2) *
              q.w() * q.x();
  dcm(2, 2) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) +
              static_cast<T>(2) * q.z() * q.z();
  return dcm;
}
/* LLA to ECEF */
Eigen::Vector3d lla2ecef(const Eigen::Vector3d &lla);
/* ECEF to LLA, using Olson's method */
Eigen::Vector3d ecef2lla(const Eigen::Vector3d &ecef);
/* ECEF to NED */
Eigen::Vector3d ecef2ned(const Eigen::Vector3d &ecef,
                         const Eigen::Vector3d &lla_ref);
/* NED to ECEF */
Eigen::Vector3d ned2ecef(const Eigen::Vector3d &ned,
                         const Eigen::Vector3d &lla_ref);
/* LLA to NED */
Eigen::Vector3d lla2ned(const Eigen::Vector3d &loc,
                        const Eigen::Vector3d &ref);
/* NED to LLA */
Eigen::Vector3d ned2lla(const Eigen::Vector3d &loc,
                        const Eigen::Vector3d &ref);
/* Converts a +/- 180 value to a 0 - 360 value */
template<typename T>
T Constrain2Pi(T ang) {
  ang = std::fmod(ang, BFS_2PI<T>);
  if (ang < static_cast<T>(0)) {
    ang += BFS_2PI<T>;
  }
  return ang;
}
/* Converts a 0 - 360 value to a +/- 180 value */
template<typename T>
T ConstrainPi(T ang) {
  if (ang > BFS_PI<T>) {
    ang -= BFS_2PI<T>;
  }
  if (ang < -BFS_PI<T>) {
    ang += BFS_2PI<T>;
  }
  return ang;
}

}  // namespace bfs

#endif  // INCLUDE_NAVIGATION_TRANSFORMS_H_
