#pragma once
#include <stdint.h>

#include "frames.h"
#include "quat_math.h"

// Multiplicative EKF: gyro propagation + accelerometer (gravity) update.
// No magnetometer, so yaw is unobservable: it is dead-reckoned from the gyro
// and its covariance grows for the whole run. See frames.h for conventions.
//
// Error state x = [dtheta(3), dbias(3)] with
//   q_true = dq(dtheta) (x) q_hat      (dtheta expressed in the body frame)
//   dbias  = b_true - b_hat
// Linearized error dynamics (derived for the Hamilton q_inertial->body used here):
//   dtheta_dot = -[w_hat]x dtheta + dbias - n_g
//   dbias_dot  = n_b
// Accelerometer measurement (unit vector): y = A(q)*ez  =>  H = [ -[y_hat]x , 0 ]

struct EkfParams {
  // TODO: placeholders. Tune from a logged stationary run on the table.
  float gyroNoise = 1.0e-3f;      // rad/s/sqrt(Hz), gyro white noise (angle random walk)
  float gyroBiasWalk = 1.0e-5f;   // rad/s^2/sqrt(Hz), bias random walk
  float accelSigma = 0.036f;      // unit-vector noise (~0.35 m/s^2 datasheet accuracy / g)
  float accelGateG = 0.15f;       // reject accel update if | |a| - 1 g | exceeds this
  float accelInflateG = 0.05f;    // R is scaled by 1 + (dev / this)^2 as |a| departs from 1 g
  float initTiltSigmaRad = 0.035f;  // ~2 deg, initial roll/pitch uncertainty
  float initYawSigmaRad = 0.009f;   // ~0.5 deg, yaw is defined as zero at start
  float minBiasSigma = 1.0e-4f;     // rad/s floor on the initial bias uncertainty
};

enum class AccelResult : uint8_t { Applied, GatedMagnitude, Singular };

class AttitudeEKF {
 public:
  explicit AttitudeEKF(const EkfParams& params = EkfParams());

  // Start from a static measurement: accel (any units) gives roll/pitch,
  // yaw is set to zero, bias comes from averaging the stationary gyro.
  bool initialize(const Vec3& accelMean, const Vec3& gyroBias, float biasSigma);

  // Propagate with one gyro sample (rad/s, body frame) over dt seconds.
  void predict(const Vec3& gyro, float dt);

  // Apply one accelerometer sample (m/s^2, body frame).
  AccelResult updateAccel(const Vec3& accel);

  Quat quaternion() const { return q_; }                 // inertial -> body
  Vec3 gyroBias() const { return b_; }                   // rad/s
  Vec3 bodyRate() const { return omega_; }               // bias-corrected, rad/s
  Vec3 eulerRad() const { return qToEuler321(q_); }      // roll, pitch, yaw
  Vec3 sigmaRad() const;                                 // 1-sigma attitude error, body axes
  Vec3 biasSigma() const;                                // 1-sigma bias error, rad/s
  bool initialized() const { return initialized_; }

 private:
  EkfParams p_;
  Quat q_{1.0f, 0.0f, 0.0f, 0.0f};
  Vec3 b_{0.0f, 0.0f, 0.0f};
  Vec3 omega_{0.0f, 0.0f, 0.0f};
  Mat<6, 6> P_;
  bool initialized_ = false;
};
