#pragma once
#include <Arduino.h>
#include <Encoder.h>

// One reaction wheel: TB9051FTG dual-PWM drive + quadrature encoder.
// Drive convention (EN high, ENB low): PWM1 duty drives OUT1 high ("+"), PWM2 duty the
// opposite polarity. setDuty() > 0 uses PWM1. Positive speed/duty sign mapping to the wheel
// axis is fixed with the per-wheel signs in control_params.h.
class WheelMotor {
 public:
  WheelMotor(int pwm1, int pwm2, int encA, int encB, float motorSign, float encoderSign)
      : pwm1_(pwm1), pwm2_(pwm2), enc_(encA, encB), motorSign_(motorSign), encSign_(encoderSign) {}

  void begin();

  // Command motor duty in [-1, 1]; applies the duty cap, slew limit and motor sign.
  // dt is the time since the previous call (s).
  void setDuty(float duty, float dt);
  void stop();  // zero output immediately (no slew)

  // Call at a fixed rate: updates the filtered wheel speed (rad/s) from encoder counts.
  void updateSpeed(float dt);

  float speed() const { return speed_; }
  float duty() const { return duty_; }

 private:
  void writePwm(float signedDuty);

  int pwm1_, pwm2_;
  Encoder enc_;
  float motorSign_, encSign_;
  float duty_ = 0.0f;   // last applied (after sign), for the slew limit
  float speed_ = 0.0f;  // rad/s, filtered
  long lastCount_ = 0;
};
