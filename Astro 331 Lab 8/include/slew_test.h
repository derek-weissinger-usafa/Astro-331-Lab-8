#pragma once
#include <stdint.h>

#include "quat_math.h"

// SLEW TEST (stretch requirement): slew through kSlewNumTargets attitudes along the shortest
// rotation (eigenaxis) from wherever the table starts, reaching each within +/-kSlewTolDeg in
// roll, pitch and yaw and holding it for kSlewHoldS.
//
// Targets are roll / pitch (absolute, from level) and yaw RELATIVE to the heading at start.
// Defaults in control_params.h; SLEW SET changes them (tilt limited to kSlewMaxTiltDeg, below the
// supervisor's kMaxTiltDeg so a small overshoot doesn't disarm).
//
// Per target:
//   Slew    q_ref(s) = exp(s * phi) (x) q_from, s: 0 -> 1 on a cosine profile (zero rate at both
//           ends), phi = shortest rotation from q_from to the target (body frame). Duration is the
//           longest of: kSlewMinS, the time at kSlewMaxRateRadS peak rate, the time at
//           kSlewMaxAccelRadS2 peak acceleration.
//   Settle  reference = target; wait until |roll, pitch, yaw error| < kSlewTolDeg continuously for
//           kSlewSettleS (gives up after kSlewSettleTimeoutS and holds anyway, marked not settled).
//   Hold    kSlewHoldS at the target; the worst error per axis is recorded. PASS = settled and
//           every axis stayed within kSlewTolDeg for the whole hold.
// The next slew starts from this target (continuous reference). After the last hold the results
// are final (Complete) and the test keeps holding the last target (phase Done) until STOP / KILL.
// While it runs, main.cpp's supervisor uses kSlewMaxTiltDeg (25 deg) instead of kMaxTiltDeg: the
// bubble has no top, so the table is kept under 25 deg.
//
// Conventions as frames.h: q is inertial->body; reference rates are body rates in the REFERENCE
// body frame (the controller rotates them into the current body frame), as in demo_spin.h.
// Errors are 3-2-1 Euler differences (deg): estimate minus target, yaw wrapped to +/-180.
class SlewTest {
 public:
  static constexpr int kMaxTargets = 4;
  enum class Phase : uint8_t { Off, Slew, Settle, Hold, Done };
  enum class Event : uint8_t { None, SlewDone, Settled, SettleTimeout, TargetDone, Complete };

  struct Target {
    float rollDeg = 0, pitchDeg = 0, yawDeg = 0;  // yaw relative to the heading at start
  };
  struct Result {
    bool settled = false;
    float settleS = 0;        // time from end of slew to settled (s)
    Vec3 maxErrDeg{0, 0, 0};  // worst |roll|, |pitch|, |yaw| error during the hold
    bool pass = false;
  };

  SlewTest();

  // Replace target i (0-based). Returns false (unchanged) if i is out of range or the tilt
  // exceeds kSlewMaxTiltDeg.
  bool setTarget(int i, float rollDeg, float pitchDeg, float yawDeg);
  const Target& target(int i) const { return targets_[i]; }

  void start(const Quat& qNow);
  Event step(float dt, const Quat& qEst);
  void abort() { phase_ = Phase::Off; }

  bool active() const { return phase_ != Phase::Off; }
  Phase phase() const { return phase_; }
  char phaseChar() const;     // - off, S slew, W settle (waiting), H hold, D done (holding last target)
  bool done() const { return phase_ == Phase::Done; }
  int index() const { return idx_; }  // current target, 0-based
  int passCount() const;
  const Result& result(int i) const { return results_[i]; }
  float slewDuration() const { return T_; }
  float slewAngleDeg() const { return norm(phi_) * kRadToDeg; }
  float elapsed() const { return elapsed_; }
  Vec3 errorDeg() const { return errDeg_; }  // latest estimate - target (roll, pitch, yaw), deg
  float refRate() const { return norm(refW_); }
  float refYawDeg() const { return qToEuler321(refQ_).z * kRadToDeg; }
  const Quat& refQ() const { return refQ_; }
  const Vec3& refW() const { return refW_; }
  const Vec3& refA() const { return refA_; }

 private:
  void beginSlew(const Quat& qFrom);
  Quat targetQuat(int i) const;
  void updateError(const Quat& qEst);

  Target targets_[kMaxTargets];
  Result results_[kMaxTargets];
  Phase phase_ = Phase::Off;
  int idx_ = 0;
  float yaw0_ = 0;     // heading at start (rad); target yaws are relative to it
  float t_ = 0;        // time in the current phase (s)
  float elapsed_ = 0;  // since start (s)
  float okT_ = 0;      // time continuously within tolerance (settle)
  Quat qFrom_{1, 0, 0, 0}, qTarget_{1, 0, 0, 0};
  Vec3 phi_{0, 0, 0};  // slew rotation vector (body frame)
  float T_ = 0;        // slew duration (s)
  Vec3 errDeg_{0, 0, 0};
  Quat refQ_{1, 0, 0, 0};
  Vec3 refW_{0, 0, 0}, refA_{0, 0, 0};
};
