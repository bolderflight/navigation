/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* NavPy developed the original Python algorithms used in this file. Brian
* Taylor converted these algorithms to C++, developed unit tests against
* NavPy generated results, and wrote the documentation. NavPy and Bolder Flight
* Systems Copyrights and licenses are below. 
*
* Copyright (c) 2014, NavPy Developers
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#ifndef NAVIGATION_SRC_EARTH_MODEL_H_  // NOLINT
#define NAVIGATION_SRC_EARTH_MODEL_H_

#include <cmath>
#include <array>
#include "units.h"  // NOLINT
#include "constants.h"  // NOLINT
#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {

/*
* Calculates the radius of curvature in the prime-vertical
* (i.e. transverse or east-west) direction
*/
inline double earthrad_transverse_m(const double lat,
                                    const AngPosUnit ang = AngPosUnit::DEG) {
  double rn = SEMI_MAJOR_AXIS_LENGTH_M / std::sqrt(1.0 - (ECC2 *
              std::pow(std::sin(convang(lat, ang, AngPosUnit::RAD)), 2.0)));
  return rn;
}

/*
* Calculates the radius of curvature in the meridonal (i.e. north-south)
* direction
*/
inline double earthrad_meridonal_m(const double lat,
                                   const AngPosUnit ang = AngPosUnit::DEG) {
  double rm = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2) / std::pow(1.0 - ECC2 *
              std::pow(std::sin(convang(lat, ang, AngPosUnit::RAD)), 2.0),
              1.5);
  return rm;
}

/*
* Calculates the radius of curvature in the transverse and meridonal directions
*/
inline std::array<double, 2> earthrad_m(const double lat,
                                        const AngPosUnit ang =
                                        AngPosUnit::DEG) {
  std::array<double, 2> ret;
  ret[0] = earthrad_transverse_m(lat, ang);
  ret[1] = earthrad_meridonal_m(lat, ang);
  return ret;
}

/*
* Calculate Latitude, Longitude, Altitude Rate given locally tangent velocity
*/
inline Eigen::Vector3d llarate(const double vn, const double ve,
                               const double vd, const double lat,
                               const double alt,
                               const AngPosUnit ang = AngPosUnit::DEG) {
  Eigen::Vector3d lla_dot;
  double Rns = earthrad_meridonal_m(lat, ang);
  double Rew = earthrad_transverse_m(lat, ang);
  lla_dot(0) = convang(vn / (Rns + alt), AngPosUnit::RAD, ang);
  lla_dot(1) = convang(ve / (Rew + alt) /
                       std::cos(convang(lat, ang, AngPosUnit::RAD)),
                       AngPosUnit::RAD, ang);
  lla_dot(2) = -vd;
  return lla_dot;
}

inline Eigen::Vector3d llarate(const Eigen::Vector3d &ned_vel,
                               const Eigen::Vector3d &lla,
                               const AngPosUnit ang = AngPosUnit::DEG) {
  return llarate(ned_vel(0), ned_vel(1), ned_vel(2), lla(0), lla(2), ang);
}

/*
* Calculate the earth rotation rate resolved on NED axis given lat
*/
inline Eigen::Vector3d earthrate(const double lat,
                                 const AngPosUnit ang = AngPosUnit::DEG) {
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  w(0) = convang(WE_RADPS * std::cos(convang(lat, ang, AngPosUnit::RAD)),
                 AngPosUnit::RAD, ang);
  w(1) = convang(-WE_RADPS * std::sin(convang(lat, ang, AngPosUnit::RAD)),
                 AngPosUnit::RAD, ang);
  return w;
}

/*
* Calculate navigation/transport rate given VN, VE, VD, lat, and alt.
* Navigation/transport rate is the angular velocity of the NED frame relative
* to the earth ECEF frame.
*/
inline Eigen::Vector3d navrate(const double vn, const double ve,
                               const double vd, const double lat,
                               const double alt,
                               const AngPosUnit ang = AngPosUnit::DEG) {
  Eigen::Vector3d w;
  double rew = earthrad_transverse_m(lat, ang);
  double rns = earthrad_meridonal_m(lat, ang);
  w(0) = convang(ve / (rew + alt), AngPosUnit::RAD, ang);
  w(1) = convang(-vn / (rns + alt), AngPosUnit::RAD, ang);
  w(2) = convang(-ve * std::tan(convang(lat, ang, AngPosUnit::RAD)) /
                 (rew + alt), AngPosUnit::RAD, ang);
  return w;
}

inline Eigen::Vector3d navrate(const Eigen::Vector3d &ned_vel,
                               const Eigen::Vector3d &lla,
                               const AngPosUnit ang = AngPosUnit::DEG) {
  return navrate(ned_vel(0), ned_vel(1), ned_vel(2), lla(0), lla(2), ang);
}

}  // namespace bfs

#endif  // NAVIGATION_SRC_EARTH_MODEL_H_ NOLINT
