#pragma once
#include <stdint.h>

#include "quat_math.h"

// Reference generator for the "level, then spin about +z" demonstration.
//
// Phases:
//   Level    reference slews smoothly from the start attitude to level (current yaw kept),
//            at <= kDemoLevelRateRadS; then waits until |roll|,|pitch| < tol for kDemoLevelHoldS.
//   SpinUp   yaw rate 0 -> target over kDemoRampS (cosine profile), staying level.
//   Hold     constant target yaw rate for kDemoHoldS.
//   SpinDown yaw rate -> 0 over kDemoRampS (also used by STOP / abort from SpinUp or Hold).
//   Settle   hold the final heading for kDemoSettleS, then Complete.
//
// Conventions as frames.h: q is inertial->body; the reference rates are body rates expressed in
// the REFERENCE body frame (the controller rotates them into the current body frame).
// Positive yaw rate = counter-clockwise seen from above (+z up).
enum class DemoPhase : uint8_t { Off, Level, SpinUp, Hold, SpinDown, Settle };

struct DemoRef {
  Quat q{1, 0, 0, 0};
  Vec3 w{0, 0, 0};  // rad/s
  Vec3 a{0, 0, 0};  // rad/s^2
};

class DemoSpin {
 public:
  enum class Event : uint8_t { None, Leveled, LevelTimeout, HoldStarted, SpinDownStarted, Settling, Complete };

  // qStart: current attitude estimate. targetRate: yaw rate to spin at (rad/s).
  void start(const Quat& qStart, float targetRate);

  // Advance by dt (s). roll/pitch (rad) are the current estimate, used to detect "level".
  Event step(float dt, float roll, float pitch);

  // STOP: from Level -> Complete immediately; from SpinUp/Hold -> SpinDown from the current
  // rate. Returns the event that results (None if already stopping or not running).
  Event requestStop();

  void abort() { phase_ = DemoPhase::Off; }

  bool active() const { return phase_ != DemoPhase::Off; }
  DemoPhase phase() const { return phase_; }
  char phaseChar() const;
  const DemoRef& ref() const { return ref_; }
  float rateRef() const { return rate_; }
  float yawRef() const { return yaw_; }
  float targetRate() const { return target_; }
  // Null-mode wheel bias to hold (rad/s): kDemoWheelBiasRadS while running, ramped to 0 over
  // the Settle phase so the wheels aren't braked from speed at the end; 0 when off.
  float wheelBias() const;

 private:
  void setLevelRef(float s, float sdot, float sddot);
  void setSpinRef();

  DemoPhase phase_ = DemoPhase::Off;
  DemoRef ref_;
  float t_ = 0;          // time in current phase (s)
  float target_ = 0;     // target yaw rate
  float rate_ = 0;       // current reference yaw rate
  float acc_ = 0;        // current reference yaw acceleration
  float yaw_ = 0;        // current reference yaw (rad)
  float downFrom_ = 0;   // rate at which SpinDown began
  // Level slew: q_ref(s) = exp(s * phi) (x) qStart, s: 0 -> 1
  Quat qStart_{1, 0, 0, 0};
  Vec3 phi_{0, 0, 0};
  float levelT_ = 0;     // slew duration
  float levelOkT_ = 0;   // time continuously within tolerance
};
