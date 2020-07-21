/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_NAVIGATION_EKF_15_STATE_H_
#define INCLUDE_NAVIGATION_EKF_15_STATE_H_

#include "types/types.h"
#include "global_defs/global_defs.h"

namespace navigation {

class Ekf15State {
 public:
  Ekf15State() {}
  template<class IMU, class GNSS>
  bool Initialize(const IMU &imu, const GNSS &gnss) {

  }
  template<class IMU>
  void TimeUpdate(const IMU &imu) {
    /* A-priori accel and rotation rate estimate */
    // accel_mps2_ = imu.accel.mps2() - accel_bias_mps2_;
    // gyro_radps_ = imu.gyro.radps() - gyro_bias_radps_;
    /* Attitude update */
    quat_ = Eigen::Quaternionf(1.0f, 0.5f * gyro_radps_(0) * dt_s_, 0.5f * gyro_radps_(1) * dt_s_, 0.5f * gyro_radps_(2) * dt_s_).normalized();
    /* Avoid quaternion sign flips */
    if (quat_.w() < 0) {
      quat_ = Eigen::Quaternionf(-quat_.w(), -quat_.x(), -quat_.y(), -quat_.z());
    }
    /* DCM */
    T_B2NED = quat2dcm(quat_).transpose();
    /* Velocity update */
    ned_vel_mps_ += dt_s_ * (T_B2NED * accel_mps2_ + grav_mps2_);
    /* Position update */
    /* Jacobian */
    /* State transition matrix */
  }
  template<class GNSS, class INS>
  void MeasurementUpdate(const GNSS &gnss, INS *ins) {

  }
 private:
  /* Time */
  float dt_s_;
  /* Accel and rotation rate */
  Eigen::Vector3f accel_mps2_;
  Eigen::Vector3f gyro_radps_;
  /* Sensor bias */
  Eigen::Vector3f accel_bias_mps2_;
  Eigen::Vector3f gyro_bias_radps_;
  /* quat */
  Eigen::Quaternionf quat_;
  /* Body to NED transform */
  Eigen::Matrix3f T_B2NED;
  /* Graviational accel in NED */
  Eigen::Vector3f grav_mps2_ = (Eigen::Vector3f() << 0.0f, 0.0f, 9.80665f).finished();
  Eigen::Vector3f ned_vel_mps_;
};

}  // namespace navigation

#endif  // INCLUDE_NAVIGATION_EKF_15_STATE_H_
