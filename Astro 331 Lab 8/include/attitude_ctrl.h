#pragma once
#include "control_params.h"
#include "quat_math.h"

// Quaternion PD (+ optional I) attitude regulator. See frames.h for conventions.
//   q_e   = q (x) q_ref*           (q, q_ref: inertial -> body)
//   th_e  = -2 sgn(w) vec(q_e)     physical body-frame angle error (rad)
//   tau_c = -Kp th_e - Kd omega - Ki int(th_e)   desired torque ON THE BODY (N m)
// Default reference is level, yaw 0 (identity quaternion).
class AttitudeController {
 public:
  AttitudeController();

  void setReference(const Quat& qRef) { qRef_ = qnormalize(qRef); }
  void reset() { integral_ = {0.0f, 0.0f, 0.0f}; }

  // omega: bias-corrected body rate (rad/s). Returns desired body torque (N m).
  Vec3 update(const Quat& q, const Vec3& omega, float dt);

  Vec3 angleError() const { return thetaE_; }  // rad, last update

 private:
  Quat qRef_{1.0f, 0.0f, 0.0f, 0.0f};
  Vec3 kp_, kd_, ki_;
  Vec3 integral_{0.0f, 0.0f, 0.0f};
  Vec3 thetaE_{0.0f, 0.0f, 0.0f};
};
