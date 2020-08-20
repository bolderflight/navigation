/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "navigation/transforms.h"
#include "navigation/constants.h"
#include "navigation/ekf_15_state.h"
#include <iostream>
#include <iomanip>

int main() {

  navigation::Ekf15State ekf;
  types::Imu imu;
  ekf.TimeUpdate(imu, 20);
  // Eigen::Vector3d lla;
  // lla(0) = 35.679862 * M_PI / 180.0;
  // lla(1) = -105.962417 * M_PI / 180.0;
  // lla(2) = 4000.0;

  // Eigen::Vector3d ecef = navigation::lla2ecef(lla);

  // Eigen::Vector3d lla2 = navigation::ecef2lla(ecef);

  // std::cout << std::setprecision(14) << lla2(0) * 180.0 / M_PI << std::endl;
  // std::cout << std::setprecision(14) << lla2(1) * 180.0 / M_PI << std::endl;
  // std::cout << std::setprecision(14) << lla2(2) << std::endl;

  // std::cout << nav::constants::ECC2 << std::endl;

  // Eigen::Vector3d ypr;
  // ypr(0) = 80 * M_PI / 180;
  // ypr(1) = 20 * M_PI / 180;
  // ypr(2) = 10 * M_PI / 180;

  // Eigen::Quaterniond q = nav::Euler_to_Quat(ypr);

  // Eigen::Vector3d eul = nav::Quat_to_Euler(q);

  // std::cout << ypr << std::endl << std::endl;
  // std::cout << eul << std::endl << std::endl;

  // Eigen::Matrix3d dcm = nav::Euler_to_Dcm(ypr);
  // Eigen::Matrix3d dcm2 = nav::Quat_to_Dcm(q);

  // Eigen::Vector3d eul2 = nav::Dcm_to_Euler(dcm);
  // Eigen::Vector3d eul3 = nav::Dcm_to_Euler(dcm2);

  // std::cout << eul2 << std::endl << std::endl;
  // std::cout << eul3 << std::endl << std::endl;

  // Eigen::Quaterniond q2 = nav::Dcm_to_Quat(dcm);
  // Eigen::Quaterniond q3 = nav::Dcm_to_Quat(dcm2);

  // // Eigen::Quaterniond q2 = nav::Dcm_to_Quat(dcm);

  // std::cout << q.w() << std::endl;
  // std::cout << q.x() << std::endl;
  // std::cout << q.y() << std::endl;
  // std::cout << q.z() << std::endl << std::endl;
  // std::cout << q2.w() << std::endl;
  // std::cout << q2.x() << std::endl;
  // std::cout << q2.y() << std::endl;
  // std::cout << q2.z() << std::endl << std::endl;
  // std::cout << q3.w() << std::endl;
  // std::cout << q3.x() << std::endl;
  // std::cout << q3.y() << std::endl;
  // std::cout << q3.z() << std::endl << std::endl;

}
