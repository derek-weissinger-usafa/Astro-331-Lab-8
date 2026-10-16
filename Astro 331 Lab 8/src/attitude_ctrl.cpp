#include "attitude_ctrl.h"

AttitudeController::AttitudeController() {
  const float wn = params::kAttBandwidthRadS;
  const float z = params::kAttDamping;
  const float* I = params::kBodyInertia;
  kp_ = {I[0] * wn * wn, I[1] * wn * wn, I[2] * wn * wn};
  kd_ = {2.0f * z * I[0] * wn, 2.0f * z * I[1] * wn, 2.0f * z * I[2] * wn};
  ki_ = {params::kAttKi, params::kAttKi, params::kAttKi};
}

static float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static Vec3 mulMat(const Mat<3, 3>& A, const Vec3& v) {
  return {A(0, 0) * v.x + A(0, 1) * v.y + A(0, 2) * v.z, A(1, 0) * v.x + A(1, 1) * v.y + A(1, 2) * v.z,
          A(2, 0) * v.x + A(2, 1) * v.y + A(2, 2) * v.z};
}

Vec3 AttitudeController::update(const Quat& q, const Vec3& omega, float dt, const Vec3& hWheels) {
  Quat qe = qmul(q, qconj(qRef_));
  if (qe.w < 0.0f) qe = {-qe.w, -qe.x, -qe.y, -qe.z};  // take the short way around
  thetaE_ = {-2.0f * qe.x, -2.0f * qe.y, -2.0f * qe.z};

  if (ki_.x != 0.0f || ki_.y != 0.0f || ki_.z != 0.0f) {
    const float lim = params::kAttIntegralClamp;
    integral_.x = clampf(integral_.x + thetaE_.x * dt, lim);
    integral_.y = clampf(integral_.y + thetaE_.y * dt, lim);
    integral_.z = clampf(integral_.z + thetaE_.z * dt, lim);
  }

  // Reference rate/accel moved from the reference body frame into the current body frame.
  const Mat<3, 3> Ae = qToMatrix(qe);
  const Vec3 wr = mulMat(Ae, wRef_);
  const Vec3 ar = mulMat(Ae, aRef_);

  const float* I = params::kBodyInertia;
  const Vec3 Iw = {I[0] * omega.x, I[1] * omega.y, I[2] * omega.z};
  const Vec3 gyro = cross(omega, Iw + hWheels);

  return {-kp_.x * thetaE_.x - kd_.x * (omega.x - wr.x) - ki_.x * integral_.x + I[0] * ar.x + gyro.x,
          -kp_.y * thetaE_.y - kd_.y * (omega.y - wr.y) - ki_.y * integral_.y + I[1] * ar.y + gyro.y,
          -kp_.z * thetaE_.z - kd_.z * (omega.z - wr.z) - ki_.z * integral_.z + I[2] * ar.z + gyro.z};
}
