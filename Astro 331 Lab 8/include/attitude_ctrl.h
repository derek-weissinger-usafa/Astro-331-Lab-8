#pragma once
#include "control_params.h"
#include "quat_math.h"

// Quaternion PD (+ optional I) attitude controller with trajectory feed-forward.
// See frames.h for conventions.
//   q_e    = q (x) q_ref*            (q, q_ref: inertial -> body); v_body = A(q_e) v_refbody
//   th_e   = -2 sgn(w) vec(q_e)      physical body-frame angle error (rad)
//   w_r    = A(q_e) w_ref,  a_r = A(q_e) a_ref   (reference rate/accel moved into the body frame)
//   tau_c  = -Kp th_e - Kd (omega - w_r) - Ki int(th_e)
//            + I a_r + omega x (I omega + h_w)       desired torque ON THE BODY (N m)
// The last term cancels gyroscopic coupling with the wheels' stored momentum h_w (body frame).
// Default reference is level, yaw 0 (identity) with zero rate.
class AttitudeController {
 public:
  AttitudeController();

  // Fixed attitude target (zero rate/accel).
  void setReference(const Quat& qRef) { setTrajectory(qRef, {0, 0, 0}, {0, 0, 0}); }
  // Moving target: wRef/aRef are expressed in the REFERENCE body frame.
  void setTrajectory(const Quat& qRef, const Vec3& wRef, const Vec3& aRef) {
    qRef_ = qnormalize(qRef);
    wRef_ = wRef;
    aRef_ = aRef;
  }
  void reset() { integral_ = {0.0f, 0.0f, 0.0f}; }

  // omega: bias-corrected body rate (rad/s). hWheels: total wheel angular momentum in the body
  // frame (N m s). Returns desired body torque (N m).
  Vec3 update(const Quat& q, const Vec3& omega, float dt, const Vec3& hWheels = {0, 0, 0});

  Vec3 angleError() const { return thetaE_; }  // rad, last update

 private:
  Quat qRef_{1.0f, 0.0f, 0.0f, 0.0f};
  Vec3 wRef_{0.0f, 0.0f, 0.0f};
  Vec3 aRef_{0.0f, 0.0f, 0.0f};
  Vec3 kp_, kd_, ki_;
  Vec3 integral_{0.0f, 0.0f, 0.0f};
  Vec3 thetaE_{0.0f, 0.0f, 0.0f};
};
