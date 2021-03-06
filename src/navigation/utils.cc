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

#include "navigation/utils.h"
#include "navigation/constants.h"
#include "Eigen/Core"
#include "Eigen/Dense"

namespace bfs {
/* Rate of change of LLA given NED velocity */
Eigen::Vector3f LlaRate(const Eigen::Vector3f &ned_vel,
                        const Eigen::Vector3d &lla) {
  Eigen::Vector3f lla_dot;
  double lat = lla(0);
  double alt = lla(2);
  double denom = fabs(1 - (E2 * sin(lat) * sin(lat)));
  double sqrt_denom = denom;
  double Rns = SEMI_MAJOR_AXIS_LENGTH_M * (1 - E2) / (denom * sqrt_denom);
  double Rew = SEMI_MAJOR_AXIS_LENGTH_M / sqrt_denom;
  lla_dot(0) = ned_vel(0) / (Rns + alt);
  lla_dot(1) = ned_vel(1) / ((Rew + alt) * cos(lat));
  lla_dot(2) = -ned_vel(2);
  return lla_dot;
}

}  // namespace bfs
