/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "navigation/utils.h"
#include "navigation/constants.h"
#include "Eigen/Core"
#include "Eigen/Dense"

namespace navigation {
/* Rate of change of LLA given NED velocity */
Eigen::Vector3f LlaRate(const Eigen::Vector3f &ned_vel, const Eigen::Vector3d &lla) {
  Eigen::Vector3f lla_dot;
  double lat = lla(0);
  double alt = lla(2);
  double denom = fabs(1 - (constants::E2 * sin(lat) * sin(lat)));
  double sqrt_denom = denom;
  double Rns = constants::SEMI_MAJOR_AXIS_LENGTH_M * (1 - constants::E2) / (denom * sqrt_denom);
  double Rew = constants::SEMI_MAJOR_AXIS_LENGTH_M / sqrt_denom;
  lla_dot(0) = ned_vel(0) / (Rns + alt);
  lla_dot(1) = ned_vel(1) / ((Rew + alt) * cos(lat));
  lla_dot(2) = -ned_vel(2);
  return lla_dot;
}

}  // namespace navigation
