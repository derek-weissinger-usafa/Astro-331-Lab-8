#include "attitude_ekf.h"

AttitudeEKF::AttitudeEKF(const EkfParams& params) : p_(params) {}

bool AttitudeEKF::initialize(const Vec3& accelMean, const Vec3& gyroBias, float biasSigma) {
  const float n = norm(accelMean);
  if (n < 1e-3f) return false;
  const Vec3 u = accelMean * (1.0f / n);  // "up" expressed in the body frame
  if (u.z < -0.99f) return false;         // upside down: shortest-arc init is singular

  // Shortest rotation taking inertial ez = (0,0,1) to u:  q = [1 + u.z, ez x u] normalized.
  // ez x u = (-u.y, u.x, 0). This leaves yaw at zero (no rotation about ez).
  q_ = qnormalize({1.0f + u.z, -u.y, u.x, 0.0f});

  // The shortest arc is not exactly Euler yaw = 0 when tilted (offset ~ roll*pitch/2).
  // Remove it so the 3-2-1 yaw angle is exactly zero at startup.
  const float yaw0 = qToEuler321(q_).z;
  q_ = qnormalize(qmul(q_, qFromRotVec({0.0f, 0.0f, yaw0})));
  b_ = gyroBias;
  omega_ = {0.0f, 0.0f, 0.0f};

  P_ = Mat<6, 6>();
  P_(0, 0) = P_(1, 1) = p_.initTiltSigmaRad * p_.initTiltSigmaRad;
  P_(2, 2) = p_.initYawSigmaRad * p_.initYawSigmaRad;
  const float bs = biasSigma > p_.minBiasSigma ? biasSigma : p_.minBiasSigma;
  for (int i = 3; i < 6; i++) P_(i, i) = bs * bs;

  initialized_ = true;
  return true;
}

void AttitudeEKF::predict(const Vec3& gyro, float dt) {
  if (!initialized_ || dt <= 0.0f) return;

  const Vec3 w = gyro - b_;
  omega_ = w;

  // Nominal state: q_{k+1} = conj(exp(w dt)) (x) q_k   (from q_dot = -1/2 [0,w] (x) q)
  q_ = qnormalize(qmul(qconj(qFromRotVec(w * dt)), q_));

  // Error-state transition  Phi = I + F dt,  F = [[-[w]x, I],[0, 0]]
  const Mat<3, 3> W = skew(w);
  Mat<6, 6> Phi = Mat<6, 6>::identity();
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) Phi(i, j) -= W(i, j) * dt;
  for (int i = 0; i < 3; i++) Phi(i, 3 + i) = dt;

  Mat<6, 6> Qd;
  const float qa = p_.gyroNoise * p_.gyroNoise * dt;
  const float qb = p_.gyroBiasWalk * p_.gyroBiasWalk * dt;
  for (int i = 0; i < 3; i++) {
    Qd(i, i) = qa;
    Qd(3 + i, 3 + i) = qb;
  }

  P_ = Phi * P_ * transpose(Phi) + Qd;
}

AccelResult AttitudeEKF::updateAccel(const Vec3& accel) {
  if (!initialized_) return AccelResult::Singular;

  const float mag = norm(accel);
  const float magG = mag / kGravity;
  const float dev = fabsf(magG - 1.0f);
  if (dev > p_.accelGateG || mag < 1e-3f) return AccelResult::GatedMagnitude;

  // Measured and predicted "up" direction in the body frame.
  const Vec3 y = accel * (1.0f / mag);
  const Mat<3, 3> A = qToMatrix(q_);
  const Vec3 yhat = {A(0, 2), A(1, 2), A(2, 2)};  // A * ez

  const Vec3 r = y - yhat;

  // H = [ -[yhat]x , 0 ]  (3x6)
  const Mat<3, 3> Sk = skew(yhat);
  Mat<3, 6> H;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) H(i, j) = -Sk(i, j);

  // Measurement noise, inflated as |a| departs from 1 g.
  const float infl = dev / p_.accelInflateG;
  const float rv = p_.accelSigma * p_.accelSigma * (1.0f + infl * infl);
  Mat<3, 3> R;
  R(0, 0) = R(1, 1) = R(2, 2) = rv;

  const Mat<6, 3> PHt = P_ * transpose(H);
  const Mat<3, 3> S = H * PHt + R;
  Mat<3, 3> Sinv;
  if (!inverse3(S, Sinv)) return AccelResult::Singular;
  const Mat<6, 3> K = PHt * Sinv;

  // Error-state correction
  float dx[6];
  const float rr[3] = {r.x, r.y, r.z};
  for (int i = 0; i < 6; i++) {
    dx[i] = 0.0f;
    for (int j = 0; j < 3; j++) dx[i] += K(i, j) * rr[j];
  }

  // Joseph-form covariance update (keeps P symmetric positive semi-definite)
  const Mat<6, 6> IKH = Mat<6, 6>::identity() - K * H;
  P_ = IKH * P_ * transpose(IKH) + K * R * transpose(K);

  // Fold the correction in multiplicatively, then reset the error state.
  q_ = qnormalize(qmul(qFromRotVec({dx[0], dx[1], dx[2]}), q_));
  b_ = b_ + Vec3{dx[3], dx[4], dx[5]};
  return AccelResult::Applied;
}

Vec3 AttitudeEKF::sigmaRad() const {
  return {sqrtf(P_(0, 0)), sqrtf(P_(1, 1)), sqrtf(P_(2, 2))};
}

Vec3 AttitudeEKF::biasSigma() const {
  return {sqrtf(P_(3, 3)), sqrtf(P_(4, 4)), sqrtf(P_(5, 5))};
}
