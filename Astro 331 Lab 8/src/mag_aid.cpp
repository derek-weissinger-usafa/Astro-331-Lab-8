#include "mag_aid.h"

#include <math.h>

#include "control_params.h"

using namespace params;

// Mean field of each spinning wheel vs its speed, fixed-table sweep 2026-10-27 (test 3: each wheel
// alone at +-100/200/300 rpm, 12 s steps). Per wheel and body axis:
//   field = const + kDirStep * sign(speed) + kSpeedSlope * (|speed| - kMagRefSpeedRadS)
// The constant is absorbed by the learned reference. The direction step is the big term
// (+-0.3-0.7 uT, i.e. 1-2 deg of heading per reversal); speed hardly matters once spinning
// (<= 0.013 uT per rad/s). Fit residual <= 0.15 uT rms (<= 0.8 deg). Wheel D's encoder was dead
// during the calibration (one channel lost), so D has no measured terms (zeros): recalibrate it
// once D is repaired (tools/mag_cal.py).
static const float kSpeedSlope[4][3] = {{-0.0031f, 0.0059f, 0.0020f},
                                        {-0.0052f, -0.0062f, -0.0035f},
                                        {0.0134f, -0.0021f, 0.0028f},
                                        {0.0f, 0.0f, 0.0f}};
static const float kDirStep[4][3] = {{0.319f, 0.332f, -0.082f},
                                     {-0.471f, 0.109f, -0.007f},
                                     {-0.717f, -0.232f, -0.005f},
                                     {0.0f, 0.0f, 0.0f}};

// A^T v: body -> inertial
static Vec3 toInertial(const Quat& q, const Vec3& v) {
  const Mat<3, 3> A = qToMatrix(q);
  return {A(0, 0) * v.x + A(1, 0) * v.y + A(2, 0) * v.z, A(0, 1) * v.x + A(1, 1) * v.y + A(2, 1) * v.z,
          A(0, 2) * v.x + A(1, 2) * v.y + A(2, 2) * v.z};
}

Vec3 MagAid::correct(const Vec3& m, const float speed[4], const bool enabled[4]) const {
  Vec3 c = m;
  for (int i = 0; i < 4; i++) {
    if (!enabled[i] || fabsf(speed[i]) < kMagMinWheelRadS) continue;
    const float over = fabsf(speed[i]) - kMagRefSpeedRadS;
    const float sg = speed[i] > 0.0f ? 1.0f : -1.0f;
    c.x -= kSpeedSlope[i][0] * over + kDirStep[i][0] * sg;
    c.y -= kSpeedSlope[i][1] * over + kDirStep[i][1] * sg;
    c.z -= kSpeedSlope[i][2] * over + kDirStep[i][2] * sg;
  }
  return c;
}

void MagAid::reset() {
  stopped_.valid = spinning_.valid = false;
  mode_ = Mode::Gated;
  settleS_ = 0.0f;
  learnS_ = 0.0f;
  learnN_ = 0;
  learnSum_ = {0, 0, 0};
}

char MagAid::modeChar() const {
  switch (mode_) {
    case Mode::Gated: return 'g';
    case Mode::LearnStopped: return 'l';
    case Mode::LearnSpinning: return 'L';
    case Mode::Stopped: return 's';
    case Mode::Spinning: return 'S';
    default: return '-';
  }
}

bool MagAid::update(const Vec3& magRaw, const long counts[4], const float speed[4], const bool enabled[4],
                    const Quat& q, float dt, Vec3& magOut, Vec3& refOut) {
  if (!enabled_) {
    mode_ = Mode::Off;
    lpInit_ = false;
    return false;
  }
  if (mode_ == Mode::Off) reset();

  // Wheel state: all enabled wheels still (and not creeping), or all spinning fast enough.
  bool still = true, fast = true, moved = false;
  for (int i = 0; i < 4; i++) {
    if (!enabled[i]) continue;
    const float w = fabsf(speed[i]);
    still = still && w < kMagStillRadS;
    fast = fast && w > kMagMinWheelRadS;
    moved = moved || counts[i] != lastCounts_[i];
    lastCounts_[i] = counts[i];
  }
  if (still && moved) still = false;  // creeping: field still changing
  const Mode want = still ? Mode::Stopped : fast ? Mode::Spinning : Mode::Gated;

  // Low-pass the corrected field (removes the shaft-rate and PWM-aliased noise).
  const Vec3 c = correct(magRaw, speed, enabled);
  if (!lpInit_) {
    lp_ = c;
    lpInit_ = true;
  }
  const float a = dt / (kMagLpfTauS + dt);
  lp_ = lp_ + (c - lp_) * a;

  const bool stateKept = (want == Mode::Stopped && (mode_ == Mode::Stopped || mode_ == Mode::LearnStopped)) ||
                         (want == Mode::Spinning && (mode_ == Mode::Spinning || mode_ == Mode::LearnSpinning));
  if (!stateKept) {
    settleS_ = 0.0f;
    learnS_ = 0.0f;
    learnN_ = 0;
    learnSum_ = {0, 0, 0};
  }
  settleS_ += dt;

  if (want == Mode::Gated || settleS_ < (want == Mode::Stopped ? kMagSettleStillS : kMagSettleSpinS)) {
    // Not yet settled: hold the mode letter of the target state only once it is usable.
    if (!stateKept) mode_ = want == Mode::Gated ? Mode::Gated : (want == Mode::Stopped ? Mode::LearnStopped
                                                                                       : Mode::LearnSpinning);
    return false;
  }

  Ref& ref = want == Mode::Stopped ? stopped_ : spinning_;
  bool match = ref.valid;
  if (match && want == Mode::Stopped) {
    for (int i = 0; i < 4; i++)
      if (enabled[i] && labs(counts[i] - ref.counts[i]) > kMagStoppedCountTol) match = false;
  }

  if (!match) {
    // Learn this state's reference with the current attitude.
    mode_ = want == Mode::Stopped ? Mode::LearnStopped : Mode::LearnSpinning;
    learnSum_ = learnSum_ + toInertial(q, lp_);
    learnN_++;
    learnS_ += dt;
    if (learnS_ >= (want == Mode::Stopped ? kMagLearnStillS : kMagLearnSpinS)) {
      ref.field = learnSum_ * (1.0f / (float)learnN_);
      for (int i = 0; i < 4; i++) ref.counts[i] = counts[i];
      ref.valid = true;
    }
    return false;
  }

  mode_ = want;
  const Vec3 mI = toInertial(q, lp_);
  residDeg_ = atan2f(ref.field.x * mI.y - ref.field.y * mI.x, ref.field.x * mI.x + ref.field.y * mI.y) * 57.29578f;
  // Gate on field strength: a horizontal field far from the reference means a disturbance the
  // model doesn't know about (a magnet nearby, a wheel state not calibrated).
  const float hm = sqrtf(mI.x * mI.x + mI.y * mI.y);
  const float hr = sqrtf(ref.field.x * ref.field.x + ref.field.y * ref.field.y);
  if (fabsf(hm - hr) > kMagHorizTolFrac * hr) {
    mode_ = Mode::Gated;
    return false;
  }
  magOut = lp_;
  refOut = ref.field;
  return true;
}
