#pragma once
#include <stdint.h>

#include "quat_math.h"

// Magnetometer heading aid: turns the raw (uncalibrated) BNO085 field into heading updates for
// the EKF (AttitudeEKF::updateMagYaw) while keeping the reaction wheels' own fields out of it.
// Calibrated on the fixed table 2026-10-27 (raw captures test_runs/MAGCAL_2026-10-27_t*.txt;
// status and open issues in TODO.md section 0):
//   * Each motor's magnets add a field that depends on where its shaft sits: a stopped wheel
//     leaves a 1-5 uT offset (= several degrees) that changes every time it stops somewhere new.
//   * A spinning wheel's field averages out (fast noise at shaft harmonics and aliased PWM ripple),
//     leaving a repeatable mean that shifts with speed (modelled per wheel in mag_aid.cpp).
//   * Slow or reversing wheels sweep the field too slowly to filter.
// So the aid works in two wheel "modes", each with its own reference field (inertial frame):
//   Stopped:  every enabled wheel still and at the encoder counts the reference was taken at.
//             Captured at the end of the startup calibration (that defines yaw 0) and re-learned
//             whenever the wheels come to rest somewhere new.
//   Spinning: every enabled wheel faster than kMinWheelRadS. Learned the first time the wheels
//             spin after a calibration, then kept (the speed model removes speed changes).
// A reference is learned by averaging A(q)^T m over kLearnS using the EKF's current attitude:
// the gyro carries yaw accurately over those few seconds, so the new reference inherits the
// current heading (only the small error accumulated since the last update). Anything else
// (a wheel slow, starting, stopping or reversing) is gated and the gyro dead-reckons through it.
// Disabled wheels (WHEEL x OFF / encoder fault) are assumed not to move; their field is part of
// both references.
// Not modelled: the table's own hard iron. The references are learned near the hold heading,
// so this only matters for large yaw changes (slews): see the TODO in mag_aid.cpp.
class MagAid {
 public:
  enum class Mode : uint8_t { Off, Gated, LearnStopped, LearnSpinning, Stopped, Spinning };

  // Per magnetometer sample. counts/speed: encoder counts and wheel speed (rad/s) per wheel;
  // enabled: which wheels are in service. Returns true (and fills magOut, refOut) when an EKF
  // heading update should be applied with this (low-passed, wheel-corrected) field.
  bool update(const Vec3& magRaw, const long counts[4], const float speed[4], const bool enabled[4],
              const Quat& q, float dt, Vec3& magOut, Vec3& refOut);

  // Startup/CALIBRATE: forget both references and learn the stopped one now (q at yaw 0).
  void reset();
  void setEnabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }

  Mode mode() const { return mode_; }
  char modeChar() const;
  float headingResidualDeg() const { return residDeg_; }  // last field heading vs reference
  Vec3 filtered() const { return lp_; }                    // low-passed corrected field (uT)

 private:
  Vec3 correct(const Vec3& m, const float speed[4], const bool enabled[4]) const;

  bool enabled_ = true;
  Mode mode_ = Mode::Off;
  Vec3 lp_{0, 0, 0};
  bool lpInit_ = false;
  float settleS_ = 0;  // time spent in the current wheel state

  struct Ref {
    bool valid = false;
    Vec3 field{0, 0, 0};  // inertial frame
    long counts[4] = {0, 0, 0, 0};
  };
  Ref stopped_, spinning_;
  Vec3 learnSum_{0, 0, 0};
  float learnS_ = 0;
  uint32_t learnN_ = 0;
  long lastCounts_[4] = {0, 0, 0, 0};
  float residDeg_ = 0;
};
