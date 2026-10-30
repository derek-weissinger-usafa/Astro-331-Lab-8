#include "wheel_motor.h"

#include "control_params.h"

void WheelMotor::begin() {
  pinMode(pwm1_, OUTPUT);
  pinMode(pwm2_, OUTPUT);
  analogWriteFrequency(pwm1_, params::kPwmFreqHz);
  analogWriteFrequency(pwm2_, params::kPwmFreqHz);
  stop();
  lastCount_ = enc_.read();
}

void WheelMotor::writePwm(float signedDuty) {
  const float full = (float)((1 << params::kPwmBits) - 1);
  const float mag = fabsf(signedDuty) * full;
  const int v = (int)(mag + 0.5f);
  if (signedDuty >= 0.0f) {
    analogWrite(pwm2_, 0);
    analogWrite(pwm1_, v);
  } else {
    analogWrite(pwm1_, 0);
    analogWrite(pwm2_, v);
  }
}

void WheelMotor::setDuty(float duty, float dt) {
  // Current limit: keep the duty within a band around the back-EMF duty (speed/free speed).
  const float emfDuty = speed_ / params::kWheelFreeSpeedRadS;
  const float band = params::kMotorCurrentLimitA / params::kMotorStallCurrentA;
  if (duty > emfDuty + band) duty = emfDuty + band;
  if (duty < emfDuty - band) duty = emfDuty - band;

  float target = duty * motorSign_;
  if (target > params::kMaxDuty) target = params::kMaxDuty;
  if (target < -params::kMaxDuty) target = -params::kMaxDuty;

  // Slew limit: smooths current steps drawn from the pack.
  const float step = params::kDutySlewPerSec * dt;
  if (target > duty_ + step) target = duty_ + step;
  if (target < duty_ - step) target = duty_ - step;

  duty_ = target;
  if (simDead_) {
    writeCoast();
  } else {
    writePwm(duty_);
  }
}

void WheelMotor::writeCoast() {
  float d = speed_ / params::kWheelFreeSpeedRadS;
  if (d > params::kMaxDuty) d = params::kMaxDuty;
  if (d < -params::kMaxDuty) d = -params::kMaxDuty;
  writePwm(d * motorSign_);
}

void WheelMotor::coast() {
  duty_ = 0.0f;
  writeCoast();
}

void WheelMotor::stop() {
  duty_ = 0.0f;
  if (simDead_) {  // a dead motor can't brake
    writeCoast();
    return;
  }
  analogWrite(pwm1_, 0);
  analogWrite(pwm2_, 0);
}

void WheelMotor::updateSpeed(float dt) {
  const long count = enc_.read();
  const long delta = count - lastCount_;
  lastCount_ = count;
  if (dt <= 0.0f) return;
  const float raw =
      encSign_ * (float)delta * (6.2831853f / params::kEncoderCountsPerRev) / dt;
  speed_ += params::kSpeedFilterAlpha * (raw - speed_);
}
