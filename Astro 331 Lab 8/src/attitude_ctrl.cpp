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

Vec3 AttitudeController::update(const Quat& q, const Vec3& omega, float dt) {
  Quat qe = qmul(q, qconj(qRef_));
  if (qe.w < 0.0f) qe = {-qe.w, -qe.x, -qe.y, -qe.z};  // take the short way around
  thetaE_ = {-2.0f * qe.x, -2.0f * qe.y, -2.0f * qe.z};

  if (ki_.x != 0.0f || ki_.y != 0.0f || ki_.z != 0.0f) {
    const float lim = params::kAttIntegralClamp;
    integral_.x = clampf(integral_.x + thetaE_.x * dt, lim);
    integral_.y = clampf(integral_.y + thetaE_.y * dt, lim);
    integral_.z = clampf(integral_.z + thetaE_.z * dt, lim);
  }

  return {-kp_.x * thetaE_.x - kd_.x * omega.x - ki_.x * integral_.x,
          -kp_.y * thetaE_.y - kd_.y * omega.y - ki_.y * integral_.y,
          -kp_.z * thetaE_.z - kd_.z * omega.z - ki_.z * integral_.z};
}
