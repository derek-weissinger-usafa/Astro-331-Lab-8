#pragma once
#include <stdint.h>

#include "quat_math.h"

// Magnetometer heading aid: turns the raw (uncalibrated) BNO085 field into heading updates for
// the EKF (AttitudeEKF::updateMagYaw) while keeping the reaction wheels' own fields out of it.
// Calibrated on the fixed table 2026-10-27 (raw captures test_runs/MAGCAL_2026-10-27_t*.txt;
// report Reports/Magnetometer_Yaw_Aid_Report_2026-10-27.md, open items TODO.md section 0):
//   * Each motor's magnets add a field that depends on where its shaft sits: a stopped wheel
//     leaves a 1-5 uT offset (= several degrees) that changes every time it stops somewhere new.
//   * A spinning wheel's field averages out (fast noise at shaft harmonics and aliased PWM ripple),
//     leaving a repeatable mean that hardly depends on speed but shifts 0.6-1.6 uT when the wheel
//     reverses. Single-wheel direction terms don't add up across wheels (1.4-2.9 deg of error in
//     mixed-direction combinations), so each direction pattern gets its own reference.
//   * Slow or reversing wheels sweep the field too slowly to filter.
// Wheel states, each with its own reference field (inertial frame):
//   Stopped:  every enabled wheel still and at the encoder counts the reference was taken at.
//             Captured at the end of the startup calibration (that defines yaw 0) and re-learned
//             whenever the wheels come to rest somewhere new.
//   Spinning: every enabled wheel faster than kMagSpinEnterRadS (low-passed speed, leaving below
//             kMagSpinExitRadS). One reference per direction pattern (sign of each enabled wheel),
//             learned the first time that pattern is seen; a small speed model removes the rest.
// A reference is learned by averaging A(q)^T m over the learn time using the EKF's current
// attitude, so it inherits the current heading. To keep that heading good, a reference is only
// learned while the aid is "fresh": an update was applied within kMagLearnMaxGapS (or the startup
// calibration just ended). Otherwise the state stays gated and the gyro dead-reckons, as it does
// through anything in between (a wheel slow, starting, stopping or reversing).
// Disabled wheels (WHEEL x OFF / encoder fault) are assumed not to move; their field is part of
// every reference.
// Not modelled: the table's own hard iron. The references are learned near the hold heading,
// so this only matters for large yaw changes (slews).
class MagAid {
 public:
  enum class Mode : uint8_t { Off, Gated, LearnStopped, LearnSpinning, Stopped, Spinning, Stale };

  // Per magnetometer sample. counts/speed: encoder counts and wheel speed (rad/s) per wheel;
  // enabled: which wheels are in service. Returns true (and fills magOut, refOut) when an EKF
  // heading update should be applied with this (low-passed, wheel-corrected) field.
  bool update(const Vec3& magRaw, const long counts[4], const float speed[4], const bool enabled[4],
              const Quat& q, float dt, Vec3& magOut, Vec3& refOut);

  // Startup/CALIBRATE: forget all references and learn the stopped one now (q at yaw 0).
  void reset();
  void setEnabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }

  Mode mode() const { return mode_; }
  // - off, g gated, l/L learning (stopped/spinning), s/S in use, x state needs a reference but
  // the aid isn't fresh enough to learn one (gyro-only until a known state returns)
  char modeChar() const;
  float headingResidualDeg() const { return residDeg_; }  // last field heading vs reference
  Vec3 filtered() const { return lp_; }                    // low-passed corrected field (uT)

 private:
  static constexpr int kSpinRefs = 16;  // one per direction pattern of the 4 wheels
  enum class WheelState : uint8_t { Gated, Stopped, Spinning };

  Vec3 correct(const Vec3& m, const float speed[4], const bool enabled[4]) const;

  bool enabled_ = true;
  Mode mode_ = Mode::Off;
  Vec3 lp_{0, 0, 0};
  bool lpInit_ = false;
  float lpSpeed_[4] = {0, 0, 0, 0};  // low-passed |speed| for the state decision
  bool spinning_ = false;            // hysteresis state
  WheelState state_ = WheelState::Gated;
  int pattern_ = -1;                 // direction pattern while spinning
  float settleS_ = 0;                // time spent in the current wheel state / pattern
  float sinceUpdateS_ = 1e9f;        // time since the last applied heading update

  struct Ref {
    bool valid = false;
    Vec3 field{0, 0, 0};  // inertial frame
    long counts[4] = {0, 0, 0, 0};
  };
  Ref stopped_, spin_[kSpinRefs];
  Vec3 learnSum_{0, 0, 0};
  float learnS_ = 0;
  uint32_t learnN_ = 0;
  long lastCounts_[4] = {0, 0, 0, 0};
  float residDeg_ = 0;
};
