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

#include "navigation.h"
#include "units.h"
#include <iostream>
// #include <iomanip>

int main() {
  // std::cout << bfs::BFS_PI<float> << std::endl;
  // std::cout << bfs::A1 << std::endl;
// Eigen::Matrix3f dcm;
// dcm << 0.4330,  0.2500, -0.8660,
//        0.1768,  0.9186,  0.3536,
//        0.8839, -0.3062,  0.3536;
// Eigen::Quaternionf q = bfs::dcm2quat(dcm);
// Eigen::Matrix3f dcm2 = bfs::quat2dcm(q);
// std::cout << q << std::endl;
// std::cout << dcm2 << std::endl;
// // std::cout << ang << std::endl;
// Eigen::Vector3d lla = {0, 45, 1000};
// Eigen::Vector3d ecef = bfs::lla2ecef(lla);
// std::cout << ecef << std::endl;

// Eigen::Vector3d ecef = {4510731, 4510731, 0};
// Eigen::Vector3d lla = bfs::ecef2lla(ecef);
// std::cout << lla << std::endl;

// Eigen::Vector3d ecef = {4510731, 4510731, 4510731};
// Eigen::Vector3d lla0 = {35, -105, 1000};
// Eigen::Vector3d lla1 = {35, -105, 1000};
// Eigen::Vector3d ecef = bfs::lla2ecef(lla1);
// Eigen::Vector3d ned = bfs::ecef2ned(ecef, lla0);
// std::cout << ned << std::endl;

// Eigen::Vector3d lla0 = {46.017, 7.750, 1673};
// Eigen::Vector3d lla = {45.976, 7.658, 4531};
// Eigen::Vector3d ned = bfs::lla2ned(lla, lla0);
// std::cout << ned << std::endl;

Eigen::Vector3d lla0 = {46.017, 7.750, 1673};
Eigen::Vector3d ned = {-4556.3, -7134.8, -2852.4};
Eigen::Vector3d lla = bfs::ned2lla(ned, lla0);

std::cout << lla << std::endl;

// Eigen::Vector3d lla = {44.532, -72.782, 1699};
// Eigen::Vector3d ecef = {1345.660, -4350.891, 4452.314};
// ecef = ecef * 1000;
// Eigen::Vector3d ned = bfs::ecef2ned(ecef, lla);
// std::cout << ned << std::endl;
// float pitch = 0.8;
// float roll = 0.7;
// float yaw = 0.5;
// // Eigen::Quaternionf q = bfs::angle2quat(yaw, pitch, roll);
// // std::cout << q << std::endl;
// // Eigen::Vector3f ang = bfs::quat2angle(q);
// // std::cout << ang << std::endl;

// Eigen::Quaternionf q;
// q.x() = 0.215509;
// q.y() = 0.432574;
// q.z() = 0.0846792;
// q.w() = 0.871358;
// Eigen::Vector3f ang = bfs::quat2eul(q);
// std::cout << ang << std::endl;

// Eigen::Quaternionf q = {0.3, 0.1, 1, 0.5};
// Eigen::Vector3f ang = bfs::quat2angle(q);
// std::cout << ang << std::endl;

  // Eigen::Vector3d p0, p1, ref, l0, l1;

  // p0 = {-27.4215, -29.3987, -6.4839};
  // p1 = {151.360, 90.000, 18.309};
  // ref = {bfs::deg2rad(35.15049), bfs::deg2rad(-106.732339), 2000.0};
  // l0 = bfs::ned2lla(p0, ref);
  // l1 = bfs::ned2lla(p1, ref);
  // std::cout << std::setprecision(14) << l0(0) * 180.0 / M_PI << std::endl;
  // std::cout << std::setprecision(14) << l0(1) * 180.0 / M_PI << std::endl;
  // std::cout << std::setprecision(14) << l0(2) << std::endl;

  // std::cout << std::setprecision(14) << l1(0) * 180.0 / M_PI << std::endl;
  // std::cout << std::setprecision(14) << l1(1) * 180.0 / M_PI << std::endl;
  // std::cout << std::setprecision(14) << l1(2) << std::endl;

  // navigation::Ekf15State ekf;
  // types::Imu imu;
  // ekf.TimeUpdate(imu, 20);
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
