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

#ifndef NAVIGATION_SRC_EKF_15_STATE_H_  // NOLINT
#define NAVIGATION_SRC_EKF_15_STATE_H_

#include "constants.h"  // NOLINT
#include "earth_model.h"  // NOLINT
#include "transforms.h"  // NOLINT
#include "utils.h"  // NOLINT
#include "units.h"  // NOLINT
#include "tilt_compass.h"  // NOLINT
#include "eigen.h"  // NOLINT
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
                  const Eigen::Vector3d &lla) {
    /* Observation matrix */
    h_.block(0, 0, 5, 5) = Eigen::Matrix<float, 5, 5>::Identity();
    /* Process noise covariance */
    rw_.block(0, 0, 3, 3) = accel_std_mps2_ * accel_std_mps2_ *
                            Eigen::Matrix<float, 3, 3>::Identity();
    rw_.block(3, 3, 3, 3) = gyro_std_radps_ * gyro_std_radps_ *
                            Eigen::Matrix<float, 3, 3>::Identity();
    rw_.block(6, 6, 3, 3) = 2.0f * accel_markov_bias_std_mps2_ *
                            accel_markov_bias_std_mps2_ / accel_tau_s_ *
                            Eigen::Matrix<float, 3, 3>::Identity();
    rw_.block(9, 9, 3, 3) = 2.0f * gyro_markov_bias_std_radps_ *
                            gyro_markov_bias_std_radps_ / gyro_tau_s_ *
                            Eigen::Matrix<float, 3, 3>::Identity();
    /* Observation noise covariance */
    r_.block(0, 0, 2, 2) = gnss_pos_ne_std_m_ * gnss_pos_ne_std_m_ *
                          Eigen::Matrix<float, 2, 2>::Identity();
    r_(2, 2) = gnss_pos_d_std_m_ * gnss_pos_d_std_m_;
    r_.block(3, 3, 2, 2) = gnss_vel_ne_std_mps_ * gnss_vel_ne_std_mps_ *
                          Eigen::Matrix<float, 2, 2>::Identity();
    r_(5, 5) = gnss_vel_d_std_mps_ * gnss_vel_d_std_mps_;
    /* Initial covariance estimate */
    p_.block(0, 0, 3, 3) = init_pos_err_std_m_ * init_pos_err_std_m_ *
                          Eigen::Matrix<float, 3, 3>::Identity();
    p_.block(3, 3, 3, 3) = init_vel_err_std_mps_ * init_vel_err_std_mps_ *
                          Eigen::Matrix<float, 3, 3>::Identity();
    p_.block(6, 6, 2, 2) = init_att_err_std_rad_ * init_att_err_std_rad_ *
                          Eigen::Matrix<float, 2, 2>::Identity();
    p_(8, 8) = init_heading_err_std_rad_ * init_heading_err_std_rad_;
    p_.block(9, 9, 3, 3) = init_accel_bias_std_mps2_ *
                          init_accel_bias_std_mps2_ *
                          Eigen::Matrix<float, 3, 3>::Identity();
    p_.block(12, 12, 3, 3) = init_gyro_bias_std_radps_ *
                            init_gyro_bias_std_radps_ *
                            Eigen::Matrix<float, 3, 3>::Identity();
    /* Markov bias matrices */
    accel_markov_bias_ = -1.0f / accel_tau_s_ *
                        Eigen::Matrix<float, 3, 3>::Identity();
    gyro_markov_bias_ = -1.0f / gyro_tau_s_ *
                        Eigen::Matrix<float, 3, 3>::Identity();
    /* Initialize position and velocity */
    ins_lla_rad_m_ = lla;
    ins_ned_vel_mps_ = ned_vel;
    /* Initialize sensor biases */
    gyro_bias_radps_ = gyro;
    /* New accelerations and rotation rates */
    ins_accel_mps2_ = accel - accel_bias_mps2_;
    ins_gyro_radps_ = gyro - gyro_bias_radps_;
    /* Initialize pitch, roll, and heading */
    ins_ypr_rad_ = TiltCompass(accel, mag);
    /* Euler to quaternion */
    quat_ = eul2quat(ins_ypr_rad_);
  }
  /* Perform a time update */
  void TimeUpdate(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
                  const float dt_s) {
    /* A-priori accel and rotation rate estimate */
    ins_accel_mps2_ = accel - accel_bias_mps2_;
    ins_gyro_radps_ = gyro - gyro_bias_radps_;
    /* Attitude update */
    delta_quat_.w() = 1.0f;
    delta_quat_.x() = 0.5f * ins_gyro_radps_(0) * dt_s;
    delta_quat_.y() = 0.5f * ins_gyro_radps_(1) * dt_s;
    delta_quat_.z() = 0.5f * ins_gyro_radps_(2) * dt_s;
    quat_ = (quat_ * delta_quat_).normalized();
    /* Avoid quaternion sign flips */
    if (quat_.w() < 0) {
      quat_ = Eigen::Quaternionf(-quat_.w(), -quat_.x(), -quat_.y(),
                                 -quat_.z());
    }
    ins_ypr_rad_ = quat2angle(quat_);
    /* Body to NED transformation from quat */
    t_b2ned = quat2dcm(quat_).transpose();
    /* Velocity update */
    ins_ned_vel_mps_ += dt_s * (t_b2ned * ins_accel_mps2_ + GRAV_NED_MPS2_);
    /* Position update */
    ins_lla_rad_m_ +=
      (dt_s * llarate(ins_ned_vel_mps_.cast<double>(),
      ins_lla_rad_m_)).cast<double>();
    /* Jacobian */
    fs_.block(0, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
    fs_(5, 2) = -2.0f * G_MPS2<float> / SEMI_MAJOR_AXIS_LENGTH_M;
    fs_.block(3, 6, 3, 3) = -2.0f * t_b2ned * Skew(ins_accel_mps2_);
    fs_.block(3, 9, 3, 3) = -t_b2ned;
    fs_.block(6, 6, 3, 3) = -Skew(ins_gyro_radps_);
    fs_.block(6, 12, 3, 3) = -0.5f * Eigen::Matrix<float, 3, 3>::Identity();
    fs_.block(9, 9, 3, 3) = accel_markov_bias_;  // Accel Markov Bias
    fs_.block(12, 12, 3, 3) = gyro_markov_bias_;  // Rotation Rate Markov Bias
    /* State transition matrix */
    phi_ = Eigen::Matrix<float, 15, 15>::Identity() + fs_ * dt_s;
    /* Process Noise Covariance (Discrete approximation) */
    gs_.block(3, 0, 3, 3) = -t_b2ned;
    gs_.block(6, 3, 3, 3) = -0.5f * Eigen::Matrix<float, 3, 3>::Identity();
    gs_.block(9, 6, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
    gs_.block(12, 9, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
    /* Discrete Process Noise */
    q_ = phi_ * dt_s * gs_ * rw_ * gs_.transpose();
    q_ = 0.5f * (q_ + q_.transpose().eval());
    /* Covariance Time Update */
    p_ = phi_ * p_ * phi_.transpose() + q_;
    p_ = 0.5f * (p_ + p_.transpose().eval());
  }
  /* Perform a measurement update */
  void MeasurementUpdate(const Eigen::Vector3f &ned_vel,
                         const Eigen::Vector3d &lla) {
    /* Y, error between Measures and Outputs */
    y_.segment(0, 3) = lla2ned(lla, ins_lla_rad_m_).cast<float>();
    y_.segment(3, 3) = ned_vel - ins_ned_vel_mps_;
    /* Innovation covariance */
    s_ = h_ * p_ * h_.transpose() + r_;
    /* Kalman gain */
    k_ = p_ * h_.transpose() * s_.inverse();
    /* Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K' */
    p_ = (Eigen::Matrix<float, 15, 15>::Identity() - k_ * h_) * p_ *
         (Eigen::Matrix<float, 15, 15>::Identity() - k_ * h_).transpose()
         + k_ * r_ * k_.transpose();
    /* State update, x = K * y */
    x_ = k_ * y_;
    /* Position update */
    double denom = fabs(1.0 - (ECC2 * sin(ins_lla_rad_m_(0)) *
                  sin(ins_lla_rad_m_(0))));
    double sqrt_denom = denom;
    double Rns = SEMI_MAJOR_AXIS_LENGTH_M * (1 - ECC2) /
                (denom * sqrt_denom);
    double Rew = SEMI_MAJOR_AXIS_LENGTH_M / sqrt_denom;
    ins_lla_rad_m_(2) -= x_(2);
    ins_lla_rad_m_(0) += x_(0) / (Rew + ins_lla_rad_m_(2));
    ins_lla_rad_m_(1) += x_(1) / (Rns + ins_lla_rad_m_(2)) /
                         cos(ins_lla_rad_m_(0));
    /* Velocity update */
    ins_ned_vel_mps_ += x_.segment(3, 3);
    /* Attitude correction */
    delta_quat_.w() = 1.0f;
    delta_quat_.x() = x_(6);
    delta_quat_.y() = x_(7);
    delta_quat_.z() = x_(8);
    quat_ = (quat_ * delta_quat_).normalized();
    ins_ypr_rad_ = quat2angle(quat_);
    /* Update biases from states */
    accel_bias_mps2_ += x_.segment(9, 3);
    gyro_bias_radps_ += x_.segment(12, 3);
    /* Update accelerometer and gyro */
    ins_accel_mps2_ -= x_.segment(9, 3);
    ins_gyro_radps_ -= x_.segment(12, 3);
  }
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

#endif  // NAVIGATION_SRC_EKF_15_STATE_H_ NOLINT
