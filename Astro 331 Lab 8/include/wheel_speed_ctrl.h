#pragma once
#include "control_params.h"

// Inner wheel speed loop: feedforward + PI on encoder speed, output is motor duty
// in [-maxDuty, +maxDuty]. No current sensing is available, so this is the
// only per-wheel loop. Anti-windup by clamping the integrator.
class WheelSpeedLoop {
 public:
  void reset() { integral_ = 0.0f; }

  // cmd, meas in rad/s. Returns duty (before the motor's slew limit).
  // freeSpeed: wheel speed per unit duty, for the feedforward term.
  float update(float cmd, float meas, float dt, float freeSpeed = params::kWheelFreeSpeedRadS) {
    const float err = cmd - meas;
    integral_ += params::kSpeedKi * err * dt;
    if (integral_ > params::kSpeedIntegralClamp) integral_ = params::kSpeedIntegralClamp;
    if (integral_ < -params::kSpeedIntegralClamp) integral_ = -params::kSpeedIntegralClamp;

    float duty = cmd / freeSpeed + params::kSpeedKp * err + integral_;
    if (duty > params::kMaxDuty) duty = params::kMaxDuty;
    if (duty < -params::kMaxDuty) duty = -params::kMaxDuty;
    return duty;
  }

 private:
  float integral_ = 0.0f;
};
