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

  // Command motor duty in [-1, 1] (wheel-axis sign convention); applies the back-EMF
  // current limit, duty cap, slew limit and motor sign.
  // dt is the time since the previous call (s).
  void setDuty(float duty, float dt);
  void stop();  // zero output immediately (no slew). EN/ENB are hard-wired, so this BRAKES.
  // Zero motor current: drive at the back-EMF duty (speed / free speed), so the wheel coasts on
  // its own friction. (A true coast needs EN or ENB on a Teensy pin.) Used for dead wheels.
  void coast();

  // FAULT TEST: simulate a dead motor (open lead / dead pack). Everything above this class still
  // sees the commanded duty, but the motor is held at zero current (coast) in every mode.
  void setSimulatedDead(bool on) { simDead_ = on; }
  bool simulatedDead() const { return simDead_; }

  // Call at a fixed rate: updates the filtered wheel speed (rad/s) from encoder counts.
  void updateSpeed(float dt);

  float speed() const { return speed_; }

  // Raw encoder counts since the last zeroCount() (same sign as speed()). Zeroing uses an
  // offset, so it doesn't disturb the speed estimate. 400 counts = 1 output turn (6.3:1).
  long count() { return (long)encSign_ * (enc_.read() - zeroOffset_); }
  void zeroCount() { zeroOffset_ = enc_.read(); }
  float duty() const { return duty_; }
  // Applied duty (after current limit, cap and slew) in the wheel-axis sign convention,
  // i.e. the same sign as speed(). Used for telemetry.
  float axisDuty() const { return duty_ * motorSign_; }

 private:
  void writePwm(float signedDuty);
  void writeCoast();

  int pwm1_, pwm2_;
  Encoder enc_;
  float motorSign_, encSign_;
  float duty_ = 0.0f;   // last applied (after sign), for the slew limit
  float speed_ = 0.0f;  // rad/s, filtered
  long lastCount_ = 0;
  long zeroOffset_ = 0;
  bool simDead_ = false;
};
