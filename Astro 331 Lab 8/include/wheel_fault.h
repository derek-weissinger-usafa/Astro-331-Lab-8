#pragma once
#include <stdint.h>

#include "quat_math.h"

// Wheel-failure handling (requirement 3). Parameters in control_params.h.
//
// DeadMotorDetector, one per wheel (main.cpp, wheelLoop, Armed only): a motor that is driven
// hard but doesn't accelerate its wheel is dead (open lead, flat or unplugged pack, failed
// driver). Drive effort
//   e = duty - speed / kWheelFreeSpeedRadS      (~ motor current / stall current)
// accelerates a working wheel at roughly kMotorAccelPerEffort * e (less friction). Every
// kDeadBlockS the speed change is compared with that prediction: a block with mean |e| >=
// kDeadEffort in which the wheel gained less than kDeadResponseFrac of the predicted speed (in
// the effort direction) is unresponsive. kDeadDetectS of unresponsive blocks in a row => dead.
// A responsive block clears the count; blocks with too little effort to judge leave it as is.
// A dead motor quickly earns large effort: its wheel coasts down on friction while the speed
// loop asks for the opposite, and the loop's integrator winds up to the current limit.
// Limits: a wheel that is never driven can't be checked (a stopped motor with no torque demand
// looks the same dead or alive), hence the wheel bias in FAULT TEST. A healthy wheel stuck by
// stiction above kDeadEffort for kDeadDetectS is reported dead too (it is not producing torque).
class DeadMotorDetector {
 public:
  void reset();
  // effort (see above) and measured speed (rad/s), both in the wheel-axis sign convention.
  // Returns true on the step the wheel is declared dead (then latches until reset()).
  bool update(float effort, float speed, float dt);
  bool dead() const { return dead_; }

 private:
  float blockT_ = 0.0f, effortSum_ = 0.0f, speed0_ = 0.0f;
  bool haveSpeed0_ = false;
  float badS_ = 0.0f;
  bool dead_ = false;
};

// FAULT TEST: fail one wheel's motor (WheelMotor::setSimulatedDead, done by main.cpp on Inject)
// without telling the controller, while whatever is Armed carries on (ARM hold, DEMO LEVEL,
// SLEW TEST, DEMO SPIN), and grade detection and recovery.
//   Wait     kFaultDelayS (counted only while Armed); wheel bias spins the wheels up. Then the
//            failure waits until the wheel turns at >= kFaultMinWheelRadS: the bias fixes only
//            the sum of the four speeds, so stored momentum can still leave one wheel idle.
//   Watch    motor dead. Waits for detection, then for the attitude error (controller angle
//            error, all three axes) to stay within +/-kFaultTolDeg for kFaultSettleS.
//   Hold     kFaultHoldS more, recording the worst error.
//   Done     PASS = right wheel detected within kFaultDetectMaxS, back within tolerance within
//            kFaultRecoverMaxS of the failure, and within tolerance through the whole hold.
// REAL mode (wheel = -1): nothing is injected; you remove power from a wheel yourself. Watch
// starts at once and times run from the detection, so detection latency isn't graded.
// The dead motor stays dead after the test (FAULT CLEAR restores it).
class FaultTest {
 public:
  enum class Phase : uint8_t { Off, Wait, Watch, Hold, Done };
  enum class Event : uint8_t { None, Inject, Recovered, Pass, Fail };

  void start(int wheel, float delayS);  // wheel 0..3, or -1 = REAL
  void abort() { phase_ = Phase::Off; }
  // Every Armed control step; errDeg = controller angle error (deg, body axes); wheelSpeed =
  // the test wheel's measured speed (rad/s).
  Event step(float dt, const Vec3& errDeg, float wheelSpeed);
  // A wheel was declared failed (dead motor or encoder fault). Returns Fail if that ends the test.
  Event onFailure(int wheel);

  bool active() const { return phase_ == Phase::Wait || phase_ == Phase::Watch || phase_ == Phase::Hold; }
  Phase phase() const { return phase_; }
  char phaseChar() const;  // - off, w wait, F watch (wheel failed), h hold, P pass / X fail (done)
  bool real() const { return real_; }
  int wheel() const { return wheel_; }  // injected wheel (REAL: the one detected, else -1)
  float wheelBias() const { return active() ? biasRadS_ : 0.0f; }
  void setWheelBias(float radS) { biasRadS_ = radS; }

  bool detected() const { return detected_; }
  float detectS() const { return detectS_; }       // failure -> detection (s)
  float recoverS() const { return recoverS_; }     // failure (REAL: detection) -> back within tol
  Vec3 peakErrDeg() const { return peak_; }        // worst |error| from failure to recovery
  Vec3 holdMaxDeg() const { return holdMax_; }     // worst |error| during the hold
  bool pass() const { return pass_; }
  const char* failReason() const { return why_; }

 private:
  Event finish(bool pass, const char* why);

  Phase phase_ = Phase::Off;
  int wheel_ = -1;
  bool real_ = false;
  float delay_ = 0.0f, t_ = 0.0f, okT_ = 0.0f, holdT_ = 0.0f;
  float biasRadS_ = 0.0f;
  bool detected_ = false, pass_ = false;
  float detectS_ = 0.0f, recoverS_ = 0.0f;
  Vec3 peak_{0, 0, 0}, holdMax_{0, 0, 0};
  const char* why_ = "";
};
