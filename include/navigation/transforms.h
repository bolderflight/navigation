/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_TRANSFORMS_H_
#define INCLUDE_NAVIGATION_TRANSFORMS_H_

#include "navigation/constants.h"
#include "Eigen/Core"
#include "Eigen/Dense"

namespace navigation {
/* Euler (3-2-1) to Direction Cosine Matrix (DCM) */
template<typename T>
Eigen::Matrix<T, 3, 3> angle2dcm(const Eigen::Matrix<T, 3, 1> &eul) {
  Eigen::Matrix<T, 3, 3> dcm;
  T psi = eul(0, 0);
  T theta = eul(1, 0);
  T phi = eul(2, 0);
  T cos_theta = cos(theta);
  T sin_theta = sin(theta);
  T cos_phi = cos(phi);
  T sin_phi = sin(phi);
  T cos_psi = cos(psi);
  T sin_psi = sin(psi);
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
  euler(0, 0) = atan2(dcm(0, 1), dcm(0, 0));
  euler(1, 0) = -asin(dcm(0, 2));
  euler(2, 0) = atan2(dcm(1, 2), dcm(2, 2));
  return euler;
}
/* Euler (3-2-1) to quaternion */
template<typename T>
Eigen::Quaternion<T> angle2quat(const Eigen::Matrix<T, 3, 1> &euler) {
  T psi = euler(0, 0) / static_cast<T>(2);
  T the = euler(1, 0) / static_cast<T>(2);
  T phi = euler(2, 0) / static_cast<T>(2);
  Eigen::Quaternion<T> q;
  q.w() = cos(psi) * cos(the) * cos(phi) + sin(psi) * sin(the) * sin(phi);
  q.x() = cos(psi) * cos(the) * sin(phi) - sin(psi) * sin(the) * cos(phi);
  q.y() = cos(psi) * sin(the) * cos(phi) + sin(psi) * cos(the) * sin(phi);
  q.z() = sin(psi) * cos(the) * cos(phi) - cos(psi) * sin(the) * sin(phi);
  return q;
}
/* Quaternion to Euler (3-2-1) */
template<typename T>
Eigen::Matrix<T, 3, 1> quat2angle(const Eigen::Quaternion<T> &q) {
  T m11 = static_cast<T>(2) * q.w() * q.w() + static_cast<T>(2) * q.x() * q.x() - static_cast<T>(1);
  T m12 = static_cast<T>(2) * q.x() * q.y() + static_cast<T>(2) * q.w() * q.z();
  T m13 = static_cast<T>(2) * q.x() * q.z() - static_cast<T>(2) * q.w() * q.y();
  T m23 = static_cast<T>(2) * q.y() * q.z() + static_cast<T>(2) * q.w() * q.x();
  T m33 = static_cast<T>(2) * q.w() * q.w() + static_cast<T>(2) * q.z() * q.z() - static_cast<T>(1);
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
  q.w() = static_cast<T>(0.5) * sqrt(static_cast<T>(1) + dcm(0, 0) + dcm(1, 1) + dcm(2, 2));
  q.x() = (dcm(1, 2) - dcm(2, 1)) / (static_cast<T>(4) * q.w());
  q.y() = (dcm(2, 0) - dcm(0, 2)) / (static_cast<T>(4) * q.w());
  q.z() = (dcm(0, 1) - dcm(1, 0)) / (static_cast<T>(4) * q.w());
  return q;
}
/* Quaternion to DCM */
template<typename T>
Eigen::Matrix<T, 3, 3> quat2dcm(const Eigen::Quaternion<T> &q) {
  Eigen::Matrix<T, 3, 3> dcm;
  dcm(0, 0) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) + static_cast<T>(2) * q.x() * q.x();
  dcm(1, 0) = static_cast<T>(2) * q.x() * q.y() - static_cast<T>(2) * q.w() * q.z();
  dcm(2, 0) = static_cast<T>(2) * q.x() * q.z() + static_cast<T>(2) * q.w() * q.y();
  dcm(0, 1) = static_cast<T>(2) * q.x() * q.y() + static_cast<T>(2) * q.w() * q.z();
  dcm(1, 1) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) + static_cast<T>(2) * q.y() * q.y();
  dcm(2, 1) = static_cast<T>(2) * q.y() * q.z() - static_cast<T>(2) * q.w() * q.x();
  dcm(0, 2) = static_cast<T>(2) * q.x() * q.z() - static_cast<T>(2) * q.w() * q.y();
  dcm(1, 2) = static_cast<T>(2) * q.y() * q.z() + static_cast<T>(2) * q.w() * q.x();
  dcm(2, 2) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) + static_cast<T>(2) * q.z() * q.z();
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
  double Rn = constants::SEMI_MAJOR_AXIS_LENGTH_M / sqrt(abs(1.0 - (constants::E2 * sin_lat * sin_lat)));
  ecef(0) = (Rn + alt) * cos_lat * cos_lon;
  ecef(1) = (Rn + alt) * cos_lat * sin_lon;
  ecef(2) = (Rn * (1.0 - constants::E2) + alt) * sin_lat;
  return ecef;
}
/* ECEF to LLA, using Olson's method */
Eigen::Vector3d ecef2lla(const Eigen::Vector3d &ecef) {
  Eigen::Vector3d lla = Eigen::Vector3d::Zero();
  static double x, y, z, zp, w2, w, z2, r2, r, s2, c2, u, v, s, ss, c, g, rg, rf, f, m, p;
  x = ecef(0);
  y = ecef(1);
  z = ecef(2);
  zp = abs(z);
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
  u = constants::A2 / r;
  v = constants::A3 - constants::A4 / r;
  if (c2 > 0.3) {
    s = (zp / r) * (1.0 + c2 * (constants::A1 + u + s2 * v) / r);
    lla(0) = asin(s);
    ss = s * s;
    c = sqrt(1.0 - ss);
  } else {
    c = (w / r) * (1.0 - s2 * (constants::A5 - u - c2 * v) / r);
    lla(0) = acos(c);
    ss = 1.0 - c * c;
    s = sqrt(ss);
  }
  g = 1.0 - constants::E2 * ss;
  rg = constants::SEMI_MAJOR_AXIS_LENGTH_M / sqrt(g);
  rf = constants::A6 * rg;
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
Eigen::Vector3d ecef2ned(const Eigen::Vector3d &ecef, const Eigen::Vector3d &lla_ref) {
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
Eigen::Vector3d ned2ecef(const Eigen::Vector3d &ned, const Eigen::Vector3d &lla_ref) {
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
Eigen::Vector3d lla2ned(const Eigen::Vector3d &loc, const Eigen::Vector3d &ref) {
  Eigen::Vector3d ecef_loc = lla2ecef(loc);
  Eigen::Vector3d ecef_ref = lla2ecef(ref);
  return ecef2ned(ecef_loc - ecef_ref, ref);
}
/* NED to LLA */
Eigen::Vector3d ned2lla(const Eigen::Vector3d &loc, const Eigen::Vector3d &ref) {
  Eigen::Vector3d ecef = ned2ecef(loc, ref);
  Eigen::Vector3d ecef_ref = lla2ecef(ref);
  ecef += ecef_ref;
  return ecef2lla(ecef);
}
}  // namespace navigation

#endif  // INCLUDE_NAVIGATION_TRANSFORMS_H_
