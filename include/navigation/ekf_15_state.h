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

#ifndef INCLUDE_NAVIGATION_EKF_15_STATE_H_
#define INCLUDE_NAVIGATION_EKF_15_STATE_H_

#include "units/units.h"
#include "Eigen/Core"
#include "Eigen/Dense"

namespace bfs {

class Ekf15State {
 public:
  Ekf15State() {}
  /* Sensor characteristics setters */
  inline void accel_std_mps2(const float val) {
    accel_std_mps2_ = val;
  }
  inline void accel_markov_bias_std_mps2(const float val) {
    accel_markov_bias_std_mps2_ = val;
  }
  inline void accel_tau_s(const float val) {
    accel_tau_s_ = val;
  }
  inline void gyro_std_radps(const float val) {
    gyro_std_radps_ = val;
  }
  inline void gyro_markov_bias_std_radps(const float val) {
    gyro_markov_bias_std_radps_ = val;
  }
  inline void gyro_tau_s(const float val) {
    gyro_tau_s_ = val;
  }
  inline void gnss_pos_ne_std_m(const float val) {
    gnss_pos_ne_std_m_ = val;
  }
  inline void gnss_pos_d_std_m(const float val) {
    gnss_pos_d_std_m_ = val;
  }
  inline void gnss_vel_ne_std_mps(const float val) {
    gnss_vel_ne_std_mps_ = val;
  }
  inline void gnss_vel_d_std_mps(const float val) {
    gnss_vel_d_std_mps_ = val;
  }
  /* Sensor characteristics getters */
  inline float accel_std_mps2() const {
    return accel_std_mps2_;
  }
  inline float accel_markov_bias_std_mps2() const {
    return accel_markov_bias_std_mps2_;
  }
  inline float accel_tau_s() const {
    return accel_tau_s_;
  }
  inline float gyro_std_radps() const {
    return gyro_std_radps_;
  }
  inline float gyro_markov_bias_std_radps() const {
    return gyro_markov_bias_std_radps_;
  }
  inline float gyro_tau_s() const {
    return gyro_tau_s_;
  }
  inline float gnss_pos_ne_std_m() const {
    return gnss_pos_ne_std_m_;
  }
  inline float gnss_pos_d_std_m() const {
    return gnss_pos_d_std_m_;
  }
  inline float gnss_vel_ne_std_mps() const {
    return gnss_vel_ne_std_mps_;
  }
  inline float gnss_vel_d_std_mps() const {
    return gnss_vel_d_std_mps_;
  }
  /* Initial covariance setters */
  inline void init_pos_err_std_m(const float val) {
    init_pos_err_std_m_ = val;
  }
  inline void init_vel_err_std_mps(const float val) {
    init_vel_err_std_mps_ = val;
  }
  inline void init_att_err_std_rad(const float val) {
    init_att_err_std_rad_ = val;
  }
  inline void init_heading_err_std_rad(const float val) {
    init_heading_err_std_rad_ = val;
  }
  inline void init_accel_bias_std_mps2(const float val) {
    init_accel_bias_std_mps2_ = val;
  }
  inline void init_gyro_bias_std_radps(const float val) {
    init_gyro_bias_std_radps_ = val;
  }
  /* Initial covariance getters */
  inline float init_pos_err_std_m() const {
    return init_pos_err_std_m_;
  }
  inline float init_vel_err_std_mps() const {
    return init_vel_err_std_mps_;
  }
  inline float init_att_err_std_rad() const {
    return init_att_err_std_rad_;
  }
  inline float init_heading_err_std_rad() const {
    return init_heading_err_std_rad_;
  }
  inline float init_accel_bias_std_mps2() const {
    return init_accel_bias_std_mps2_;
  }
  inline float init_gyro_bias_std_radps() const {
    return init_gyro_bias_std_radps_;
  }
  /* Initialize the EKF states */
  void Initialize(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
                  const Eigen::Vector3f &mag, const Eigen::Vector3f &ned_vel,
                  const Eigen::Vector3d &lla);
  /* Perform a time update */
  void TimeUpdate(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
                  const float dt_s);
  /* Perform a measurement update */
  void MeasurementUpdate(const Eigen::Vector3f &ned_vel,
                         const Eigen::Vector3d &lla);
  /* EKF data */
  inline Eigen::Vector3f accel_bias_mps2() const {
    return accel_bias_mps2_;
  }
  inline Eigen::Vector3f gyro_bias_radps() const {
    return gyro_bias_radps_;
  }
  inline Eigen::Vector3f accel_mps2() const {
    return ins_accel_mps2_;
  }
  inline Eigen::Vector3f gyro_radps() const {
    return ins_gyro_radps_;
  }
  inline Eigen::Vector3f ned_vel_mps() const {
    return ins_ned_vel_mps_;
  }
  inline float yaw_rad() const {
    return ins_ypr_rad_(0);
  }
  inline float pitch_rad() const {
    return ins_ypr_rad_(1);
  }
  inline float roll_rad() const {
    return ins_ypr_rad_(2);
  }
  inline Eigen::Vector3d lla_rad_m() const {
    return ins_lla_rad_m_;
  }
  inline double lat_rad() const {
    return ins_lla_rad_m_(0);
  }
  inline double lon_rad() const {
    return ins_lla_rad_m_(1);
  }
  inline double alt_m() const {
    return ins_lla_rad_m_(2);
  }

 private:
  /*
  * Sensor characteristics - accel and gyro are modeled with a
  * Gauss-Markov model.
  */
  /* Standard deviation of accel noise */
  float accel_std_mps2_ = 0.05f;
  /* Standard deviation of accel Markov bias */
  float accel_markov_bias_std_mps2_ = 0.01f;
  /* Accel correlation time */
  float accel_tau_s_ = 100.0f;
  Eigen::Matrix3f accel_markov_bias_ = -1.0f / accel_tau_s_ *
                                       Eigen::Matrix<float, 3, 3>::Identity();
  /* Standard deviation of gyro noise */
  float gyro_std_radps_ = 0.00175f;
  /* Standard deviation of gyro Markov bias */
  float gyro_markov_bias_std_radps_ = 0.00025f;
  /* Gyro correlation time */
  float gyro_tau_s_ = 50.0f;
  Eigen::Matrix3f gyro_markov_bias_ = -1.0f / gyro_tau_s_ *
                                      Eigen::Matrix<float, 3, 3>::Identity();
  /* Standard deviation of the GNSS North and East position measurement */
  float gnss_pos_ne_std_m_ = 3.0f;
  /* Standard deviation of the GNSS Down position estimate */
  float gnss_pos_d_std_m_ = 6.0f;
  /* Standard deviation of the GNSS North and East velocity measurement */
  float gnss_vel_ne_std_mps_ = 0.5f;
  /* Standard deviation of the GNSS Down velocity measurement */
  float gnss_vel_d_std_mps_ = 1.0f;
  /*
  * Initial set of covariances
  */
  /* Standard deviation of the initial position error */
  float init_pos_err_std_m_ = 10.0f;
  /* Standard deviation of the initial velocity error */
  float init_vel_err_std_mps_ = 1.0f;
  /* Standard deviation of the initial attitude error */
  float init_att_err_std_rad_ = 0.34906f;
  /* Standard deviation of the initial heading error */
  float init_heading_err_std_rad_ = 3.14159f;
  /* Standard deviation of the initial accel bias */
  float init_accel_bias_std_mps2_ = 0.9810f;
  /* Standard deviation of the initial gyro bias */
  float init_gyro_bias_std_radps_ = 0.01745f;
  /*
  * Kalman filter matrices
  */
  /* Observation matrix */
  Eigen::Matrix<float, 6, 15> h_ = Eigen::Matrix<float, 6, 15>::Zero();
  /* Covariance of the observation noise */
  Eigen::Matrix<float, 6, 6> r_ = Eigen::Matrix<float, 6, 6>::Zero();
  /* Covariance of the Sensor Noise */
  Eigen::Matrix<float, 12, 12> rw_ = Eigen::Matrix<float, 12, 12>::Zero();
  /* Process Noise Covariance (Discrete approximation) */
  Eigen::Matrix<float, 15, 12> gs_ = Eigen::Matrix<float, 15, 12>::Zero();
  /* Innovation covariance */
  Eigen::Matrix<float, 6, 6> s_ = Eigen::Matrix<float, 6, 6>::Zero();
  /* Covariance estimate */
  Eigen::Matrix<float, 15, 15> p_ = Eigen::Matrix<float, 15, 15>::Zero();
  /* Discrete Process Noise */
  Eigen::Matrix<float, 15, 15> q_ = Eigen::Matrix<float, 15, 15>::Zero();
  /* Kalman gain */
  Eigen::Matrix<float, 15, 6> k_ = Eigen::Matrix<float, 15, 6>::Zero();
  /* Jacobian (state update matrix) */
  Eigen::Matrix<float, 15, 15> fs_ = Eigen::Matrix<float, 15, 15>::Zero();
  /* State transition */
  Eigen::Matrix<float, 15, 15> phi_ = Eigen::Matrix<float, 15, 15>::Zero();
  /* Error between measures and outputs */
  Eigen::Matrix<float, 6, 1> y_ = Eigen::Matrix<float, 6, 1>::Zero();
  /* State matrix */
  Eigen::Matrix<float, 15, 1> x_ = Eigen::Matrix<float, 15, 1>::Zero();
  /*
  * Constants
  */
  /* Graviational accel in NED */
  Eigen::Vector3f GRAV_NED_MPS2_ =
    (Eigen::Vector3f() << 0.0f, 0.0f, G_MPS2<float>).finished();
  /*
  * Intermediates
  */
  /* Body to NED transform */
  Eigen::Matrix3f t_b2ned;
  /* Acceleration bias, m/s/s */
  Eigen::Vector3f accel_bias_mps2_ = Eigen::Vector3f::Zero();
  /* Rotation rate bias, rad/s */
  Eigen::Vector3f gyro_bias_radps_;
  /* Normalized accel */
  Eigen::Vector3f accel_norm_mps2_;
  /* Normalized mag */
  Eigen::Vector3f mag_norm_mps2_;
  /* Quaternion update */
  Eigen::Quaternionf delta_quat_;
  /* Quaternion */
  Eigen::Quaternionf quat_;
  /*
  * Data
  */
  Eigen::Vector3f ins_accel_mps2_;
  Eigen::Vector3f ins_gyro_radps_;
  Eigen::Vector3f ins_ypr_rad_;
  Eigen::Vector3f ins_ned_vel_mps_;
  Eigen::Vector3d ins_lla_rad_m_;
};

}  // namespace bfs

#endif  // INCLUDE_NAVIGATION_EKF_15_STATE_H_
