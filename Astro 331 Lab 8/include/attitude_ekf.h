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
//
// FUTURE (gated magnetometer, only if measured yaw drift is too large): add updateMag()
// with the same unit-vector update as updateAccel (v_ref = local field direction), apply it
// only when the wheels are quiet and the field magnitude is consistent. The uncalibrated
// magnetometer is already read (for the data log) in imu_bno085.cpp. Not fused now.

struct EkfParams {
  // gyroNoise / accelSigma: still the original placeholders (static tests suggest ~3e-4 and
  // ~0.01; retune separately).
  float gyroNoise = 1.0e-3f;      // rad/s/sqrt(Hz), gyro white noise (angle random walk)
  // Bias settings tightened 2026-10-16 from the static tests (STATIC_0001 / STATIC_0005): the
  // Allan deviation shows no bias random walk within 30 min (floor ~2 deg/h ~ 1e-5 rad/s).
  // With the old 1e-5 walk and 1e-4 floor, the accelerometer (via the 2.3 deg table tilt)
  // dragged the z bias from the calibrated -0.018 deg/s to ~-0.0125 deg/s => ~24 deg/h of yaw
  // drift instead of ~4.
  float gyroBiasWalk = 1.0e-6f;   // rad/s^2/sqrt(Hz), x/y bias random walk
  float gyroBiasWalkZ = 1.0e-7f;  // rad/s^2/sqrt(Hz), z bias random walk
  // z bias is not observable from gravity when level and is only weakly (and wrongly) pulled by
  // the accelerometer when tilted, so by default it comes only from the static calibration.
  bool lockZBias = true;
  float accelSigma = 0.036f;      // unit-vector noise (~0.35 m/s^2 datasheet accuracy / g)
  float accelGateG = 0.15f;       // reject accel update if | |a| - 1 g | exceeds this
  float accelInflateG = 0.05f;    // R is scaled by 1 + (dev / this)^2 as |a| departs from 1 g
  float initTiltSigmaRad = 0.035f;  // ~2 deg, initial roll/pitch uncertainty
  float initYawSigmaRad = 0.009f;   // ~0.5 deg, yaw is defined as zero at start
  float minBiasSigma = 2.0e-5f;     // rad/s floor on the initial bias uncertainty (60 s cal ~1.5e-5)
};

enum class AccelResult : uint8_t { Applied, GatedMagnitude, Singular };
enum class MagResult : uint8_t { Applied, Gated, Singular };

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

  // Heading-only magnetometer update. magBody: field (any units) with the wheel/table
  // disturbances already removed (mag_aid.h); refInertial: the same field in the inertial frame
  // at yaw 0 (captured at calibration). Only the horizontal directions are compared, the
  // correction acts on yaw alone (never roll/pitch), and it is the only update that moves the
  // z gyro bias. sigmaRad: 1-sigma heading noise per sample.
  MagResult updateMagYaw(const Vec3& magBody, const Vec3& refInertial, float sigmaRad);
  float lastMagInnovRad() const { return lastMagInnovRad_; }  // last heading residual
  // Test hook (KICK): rotate the estimate about inertial z by rad (covariance unchanged).
  void perturbYaw(float rad);

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
  float lastMagInnovRad_ = 0.0f;
};
