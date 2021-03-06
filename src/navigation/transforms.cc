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

#include "navigation/transforms.h"
#include "navigation/constants.h"
#include "Eigen/Core"
#include "Eigen/Dense"

namespace bfs {

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

}  // namespace bfs
