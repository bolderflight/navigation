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

#ifndef NAVIGATION_SRC_TRANSFORMS_H_  // NOLINT
#define NAVIGATION_SRC_TRANSFORMS_H_

#include "units.h"  // NOLINT
#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {
/* Convert rotation angles to direction cosine matrix */
template<typename T>
Eigen::Matrix<T, 3, 3> angle2dcm(const T rot1, const T rot2, const T rot3,
                                 const AngPosUnit ang = AngPosUnit::RAD) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  Eigen::Matrix<T, 3, 3> dcm;
  T cos_rot1 = std::cos(convang(rot1, ang, AngPosUnit::RAD));
  T sin_rot1 = std::sin(convang(rot1, ang, AngPosUnit::RAD));
  T cos_rot2 = std::cos(convang(rot2, ang, AngPosUnit::RAD));
  T sin_rot2 = std::sin(convang(rot2, ang, AngPosUnit::RAD));
  T cos_rot3 = std::cos(convang(rot3, ang, AngPosUnit::RAD));
  T sin_rot3 = std::sin(convang(rot3, ang, AngPosUnit::RAD));
  dcm(0, 0) = cos_rot2 * cos_rot1;
  dcm(1, 0) = -cos_rot3 * sin_rot1 + sin_rot3 * sin_rot2 * cos_rot1;
  dcm(2, 0) = sin_rot3 * sin_rot1 + cos_rot3 * sin_rot2 * cos_rot1;
  dcm(0, 1) = cos_rot2 * sin_rot1;
  dcm(1, 1) = cos_rot3 * cos_rot1 + sin_rot3 * sin_rot2 * sin_rot1;
  dcm(2, 1) = -sin_rot3 * cos_rot1 + cos_rot3 * sin_rot2 * sin_rot1;
  dcm(0, 2) = -sin_rot2;
  dcm(1, 2) = sin_rot3 * cos_rot2;
  dcm(2, 2) = cos_rot3 * cos_rot2;
  return dcm;
}
/* Convert Euler angles to direction cosine matrix */
template<typename T>
Eigen::Matrix<T, 3, 3> eul2dcm(const Eigen::Matrix<T, 3, 1> &eul,
                               const AngPosUnit ang = AngPosUnit::RAD) {
  return angle2dcm(eul(0), eul(1), eul(2), ang);
}
/* Create rotation angles from direction cosine matrix*/
template<typename T>
Eigen::Matrix<T, 3, 1> dcm2angle(const Eigen::Matrix<T, 3, 3> &dcm,
                                 const AngPosUnit ang = AngPosUnit::RAD) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  Eigen::Matrix<T, 3, 1> an;
  an(0, 0) = convang(std::atan2(dcm(0, 1), dcm(0, 0)), AngPosUnit::RAD, ang);
  an(1, 0) = convang(-std::asin(dcm(0, 2)), AngPosUnit::RAD, ang);
  an(2, 0) = convang(std::atan2(dcm(1, 2), dcm(2, 2)), AngPosUnit::RAD, ang);
  return an;
}
/* Create Euler angles from direction cosine matrix */
template<typename T>
Eigen::Matrix<T, 3, 1> dcm2eul(const Eigen::Matrix<T, 3, 3> &dcm,
                               const AngPosUnit ang = AngPosUnit::RAD) {
  return dcm2angle(dcm, ang);
}
/* Convert rotation angles to quaternion */
template<typename T>
Eigen::Quaternion<T> angle2quat(const T rot1, const T rot2, const T rot3,
                                const AngPosUnit a = AngPosUnit::RAD) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  Eigen::Quaternion<T> q;
  T cos_rot1 = std::cos(convang(rot1 / static_cast<T>(2), a, AngPosUnit::RAD));
  T sin_rot1 = std::sin(convang(rot1 / static_cast<T>(2), a, AngPosUnit::RAD));
  T cos_rot2 = std::cos(convang(rot2 / static_cast<T>(2), a, AngPosUnit::RAD));
  T sin_rot2 = std::sin(convang(rot2 / static_cast<T>(2), a, AngPosUnit::RAD));
  T cos_rot3 = std::cos(convang(rot3 / static_cast<T>(2), a, AngPosUnit::RAD));
  T sin_rot3 = std::sin(convang(rot3 / static_cast<T>(2), a, AngPosUnit::RAD));
  q.w() = cos_rot1 * cos_rot2 * cos_rot3 + sin_rot1 * sin_rot2 * sin_rot3;
  q.x() = cos_rot1 * cos_rot2 * sin_rot3 - sin_rot1 * sin_rot2 * cos_rot3;
  q.y() = cos_rot1 * sin_rot2 * cos_rot3 + sin_rot1 * cos_rot2 * sin_rot3;
  q.z() = sin_rot1 * cos_rot2 * cos_rot3 - cos_rot1 * sin_rot2 * sin_rot3;
  return q;
}
/* Convert Euler angles to quaternion */
template<typename T>
Eigen::Quaternion<T> eul2quat(const Eigen::Matrix<T, 3, 1> &eul,
                              const AngPosUnit a = AngPosUnit::RAD) {
  return angle2quat(eul(0), eul(1), eul(2), a);
}
/* Convert quaternion to rotation angles */
template<typename T>
Eigen::Matrix<T, 3, 1> quat2angle(const Eigen::Quaternion<T> &q,
                                  const AngPosUnit ang = AngPosUnit::RAD) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  Eigen::Matrix<T, 3, 1> angle;
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
  angle(0, 0) = convang(std::atan2(m12, m11), AngPosUnit::RAD, ang);
  angle(1, 0) = convang(std::asin(-m13), AngPosUnit::RAD, ang);
  angle(2, 0) = convang(std::atan2(m23, m33), AngPosUnit::RAD, ang);
  return angle;
}
/* Convert quaternion to Euler angles */
template<typename T>
Eigen::Matrix<T, 3, 1> quat2eul(const Eigen::Quaternion<T> &q,
                                const AngPosUnit ang = AngPosUnit::RAD) {
  return quat2angle(q, ang);
}

/* Convert direction cosine matrix to quaternion */
template<typename T>
Eigen::Quaternion<T> dcm2quat(const Eigen::Matrix<T, 3, 3> &dcm) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  Eigen::Quaternion<T> q;
  q.w() = static_cast<T>(0.5) * std::sqrt(static_cast<T>(1) + dcm(0, 0) +
          dcm(1, 1) + dcm(2, 2));
  q.x() = (dcm(1, 2) - dcm(2, 1)) / (static_cast<T>(4) * q.w());
  q.y() = (dcm(2, 0) - dcm(0, 2)) / (static_cast<T>(4) * q.w());
  q.z() = (dcm(0, 1) - dcm(1, 0)) / (static_cast<T>(4) * q.w());
  return q;
}
/* Convert quaternion to direction cosine matrix */
template<typename T>
Eigen::Matrix<T, 3, 3> quat2dcm(const Eigen::Quaternion<T> &q) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
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
Eigen::Vector3d lla2ecef(const Eigen::Vector3d &lla) {
  Eigen::Vector3d ecef;
  double sin_lat = sin(lla(0));
  double cos_lat = cos(lla(0));
  double cos_lon = cos(lla(1));
  double sin_lon = sin(lla(1));
  double alt = lla(2);
  double Rn = SEMI_MAJOR_AXIS_LENGTH_M /
              sqrt(fabs(1.0 - (E2 * sin_lat * sin_lat)));
  ecef(0) = (Rn + alt) * cos_lat * cos_lon;
  ecef(1) = (Rn + alt) * cos_lat * sin_lon;
  ecef(2) = (Rn * (1.0 - E2) + alt) * sin_lat;
  return ecef;
}

/* ECEF to LLA, using Olson's method */
Eigen::Vector3d ecef2lla(const Eigen::Vector3d &ecef) {
  Eigen::Vector3d lla = Eigen::Vector3d::Zero();
  static double x, y, z, zp, w2, w, z2, r2, r, s2, c2, u,
                v, s, ss, c, g, rg, rf, f, m, p;
  x = ecef(0);
  y = ecef(1);
  z = ecef(2);
  zp = fabs(z);
  w2 = x * x + y * y;
  w = sqrt(w2);
  z2 = z * z;
  r2 = w2 + z2;
  r = sqrt(r2);
  if (r < 100000.0) {
    return lla;
  }
  lla(1) = atan2(y, x);
  s2 = z2 / r2;
  c2 = w2 / r2;
  u = A2 / r;
  v = A3 - A4 / r;
  if (c2 > 0.3) {
    s = (zp / r) * (1.0 + c2 * (A1 + u + s2 * v) / r);
    lla(0) = asin(s);
    ss = s * s;
    c = sqrt(1.0 - ss);
  } else {
    c = (w / r) * (1.0 - s2 * (A5 - u - c2 * v) / r);
    lla(0) = acos(c);
    ss = 1.0 - c * c;
    s = sqrt(ss);
  }
  g = 1.0 - E2 * ss;
  rg = SEMI_MAJOR_AXIS_LENGTH_M / sqrt(g);
  rf = A6 * rg;
  u = w - rg * c;
  v = zp - rf  * s;
  f = c * u + s * v;
  m = c * v - s * u;
  p = m / (rf / g + f);
  lla(0) = lla(0) + p;
  lla(2) = f + m * p / 2.0;
  if (z < 0) {
    lla(0) = -1 * lla(0);
  }
  return lla;
}

/* ECEF to NED */
Eigen::Vector3d ecef2ned(const Eigen::Vector3d &ecef,
                         const Eigen::Vector3d &lla_ref) {
  Eigen::Matrix3d R;
  R(0, 0) = -sin(lla_ref(0)) * cos(lla_ref(1));
  R(0, 1) = -sin(lla_ref(0)) * sin(lla_ref(1));
  R(0, 2) = cos(lla_ref(0));
  R(1, 0) = -sin(lla_ref(1));
  R(1, 1) = cos(lla_ref(1));
  R(1, 2) = 0;
  R(2, 0) = -cos(lla_ref(0)) * cos(lla_ref(1));
  R(2, 1) = -cos(lla_ref(0)) * sin(lla_ref(1));
  R(2, 2) = -sin(lla_ref(0));
  return R * ecef;
}

/* NED to ECEF */
Eigen::Vector3d ned2ecef(const Eigen::Vector3d &ned,
                         const Eigen::Vector3d &lla_ref) {
  Eigen::Matrix3d R;
  R(0, 0) = -sin(lla_ref(0)) * cos(lla_ref(1));
  R(0, 1) = -sin(lla_ref(0)) * sin(lla_ref(1));
  R(0, 2) = cos(lla_ref(0));
  R(1, 0) = -sin(lla_ref(1));
  R(1, 1) = cos(lla_ref(1));
  R(1, 2) = 0;
  R(2, 0) = -cos(lla_ref(0)) * cos(lla_ref(1));
  R(2, 1) = -cos(lla_ref(0)) * sin(lla_ref(1));
  R(2, 2) = -sin(lla_ref(0));
  return R.transpose() * ned;
}

/* LLA to NED */
Eigen::Vector3d lla2ned(const Eigen::Vector3d &loc,
                        const Eigen::Vector3d &ref) {
  Eigen::Vector3d ecef_loc = lla2ecef(loc);
  Eigen::Vector3d ecef_ref = lla2ecef(ref);
  return ecef2ned(ecef_loc - ecef_ref, ref);
}

/* NED to LLA */
Eigen::Vector3d ned2lla(const Eigen::Vector3d &loc,
                        const Eigen::Vector3d &ref) {
  Eigen::Vector3d ecef = ned2ecef(loc, ref);
  Eigen::Vector3d ecef_ref = lla2ecef(ref);
  ecef += ecef_ref;
  return ecef2lla(ecef);
}

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

#endif  // NAVIGATION_SRC_TRANSFORMS_H_ NOLINT
