#include "mag_aid.h"

#include <math.h>

#include "control_params.h"

using namespace params;

// Mean field of each spinning wheel vs its speed, fixed-table sweep 2026-10-27 (test 3: each wheel
// alone at +-100/200/300 rpm, 12 s steps). Per wheel and body axis:
//   field = const + kDirStep * sign(speed) + kSpeedSlope * (|speed| - kMagRefSpeedRadS)
// The constant is absorbed by the learned reference. The direction step is the big term
// (+-0.3-0.7 uT, i.e. 1-2 deg of heading per reversal); speed hardly matters once spinning
// (<= 0.013 uT per rad/s). Fit residual <= 0.15 uT rms (<= 0.8 deg). Since each direction
// pattern now has its own reference, kDirStep only matters within a pattern (where it is
// constant); it is kept as the measured record. Wheel D's encoder was dead during the
// calibration (one channel lost), so D has no measured terms (zeros): recalibrate it once D is
// repaired (tools/mag_cal.py).
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
    if (!enabled[i] || fabsf(speed[i]) < kMagSpinExitRadS) continue;
    const float over = fabsf(speed[i]) - kMagRefSpeedRadS;
    const float sg = speed[i] > 0.0f ? 1.0f : -1.0f;
    c.x -= kSpeedSlope[i][0] * over + kDirStep[i][0] * sg;
    c.y -= kSpeedSlope[i][1] * over + kDirStep[i][1] * sg;
    c.z -= kSpeedSlope[i][2] * over + kDirStep[i][2] * sg;
  }
  return c;
}

void MagAid::reset() {
  stopped_.valid = false;
  for (int k = 0; k < kSpinRefs; k++) spin_[k].valid = false;
  mode_ = Mode::Gated;
  state_ = WheelState::Gated;
  pattern_ = -1;
  settleS_ = 0.0f;
  learnS_ = 0.0f;
  learnN_ = 0;
  learnSum_ = {0, 0, 0};
  sinceUpdateS_ = 0.0f;  // yaw was just defined: learning the stopped reference is allowed
}

char MagAid::modeChar() const {
  switch (mode_) {
    case Mode::Gated: return 'g';
    case Mode::LearnStopped: return 'l';
    case Mode::LearnSpinning: return 'L';
    case Mode::Stopped: return 's';
    case Mode::Spinning: return 'S';
    case Mode::Stale: return 'x';
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
  if (mode_ == Mode::Off) {
    reset();
    sinceUpdateS_ = 1e9f;  // re-enabled mid-run: don't trust the current yaw for learning
  }
  sinceUpdateS_ += dt;

  // Wheel state from low-passed speeds, with hysteresis on "spinning" (wheel C's speed loop
  // oscillates +-30-90 rpm, which made a raw threshold flicker and restart the settle).
  const float as = dt / (kMagSpeedLpfTauS + dt);
  bool still = true, allAbove = true, anyBelow = false, moved = false;
  int pattern = 0;
  for (int i = 0; i < 4; i++) {
    if (!enabled[i]) continue;
    const float w = fabsf(speed[i]);
    lpSpeed_[i] += as * (w - lpSpeed_[i]);
    still = still && w < kMagStillRadS;
    allAbove = allAbove && lpSpeed_[i] > kMagSpinEnterRadS;
    anyBelow = anyBelow || lpSpeed_[i] < kMagSpinExitRadS;
    if (speed[i] > 0.0f) pattern |= 1 << i;
    moved = moved || counts[i] != lastCounts_[i];
    lastCounts_[i] = counts[i];
  }
  if (still && moved) still = false;  // creeping: field still changing
  if (allAbove) spinning_ = true;
  if (anyBelow) spinning_ = false;
  const WheelState ws = still ? WheelState::Stopped : spinning_ ? WheelState::Spinning : WheelState::Gated;

  // Low-pass the corrected field (removes the shaft-rate and PWM-aliased noise).
  const Vec3 c = correct(magRaw, speed, enabled);
  if (!lpInit_) {
    lp_ = c;
    lpInit_ = true;
  }
  const float a = dt / (kMagLpfTauS + dt);
  lp_ = lp_ + (c - lp_) * a;

  // A new wheel state or direction pattern restarts the settle and any learning in progress.
  // (A pattern change without dropping below kMagSpinExitRadS can't happen: reversing passes 0.)
  const int pat = ws == WheelState::Spinning ? pattern : -1;
  if (ws != state_ || pat != pattern_) {
    state_ = ws;
    pattern_ = pat;
    settleS_ = 0.0f;
    learnS_ = 0.0f;
    learnN_ = 0;
    learnSum_ = {0, 0, 0};
  }
  settleS_ += dt;

  if (ws == WheelState::Gated) {
    mode_ = Mode::Gated;
    return false;
  }
  const bool isStopped = ws == WheelState::Stopped;
  if (settleS_ < (isStopped ? kMagSettleStillS : kMagSettleSpinS)) {
    mode_ = Mode::Gated;
    return false;
  }

  Ref& ref = isStopped ? stopped_ : spin_[pattern & (kSpinRefs - 1)];
  bool match = ref.valid;
  if (match && isStopped) {
    for (int i = 0; i < 4; i++)
      if (enabled[i] && labs(counts[i] - ref.counts[i]) > kMagStoppedCountTol) match = false;
  }

  if (!match) {
    // Learn this state's reference with the current attitude, but only while the yaw is fresh
    // (learning restarted with no recent update would bake the drift into the reference).
    if (learnN_ == 0 && sinceUpdateS_ > kMagLearnMaxGapS) {
      mode_ = Mode::Stale;
      return false;
    }
    mode_ = isStopped ? Mode::LearnStopped : Mode::LearnSpinning;
    learnSum_ = learnSum_ + toInertial(q, lp_);
    learnN_++;
    learnS_ += dt;
    if (learnS_ >= (isStopped ? kMagLearnStillS : kMagLearnSpinS)) {
      ref.field = learnSum_ * (1.0f / (float)learnN_);
      for (int i = 0; i < 4; i++) ref.counts[i] = counts[i];
      ref.valid = true;
    }
    return false;
  }

  const Vec3 mI = toInertial(q, lp_);
  residDeg_ = atan2f(ref.field.x * mI.y - ref.field.y * mI.x, ref.field.x * mI.x + ref.field.y * mI.y) * 57.29578f;
  // Gate on field strength: a horizontal field far from the reference means a disturbance the
  // model doesn't know about (a magnet nearby, a wheel state not calibrated). Skips the sample
  // only; the settle is not restarted.
  const float hm = sqrtf(mI.x * mI.x + mI.y * mI.y);
  const float hr = sqrtf(ref.field.x * ref.field.x + ref.field.y * ref.field.y);
  if (fabsf(hm - hr) > kMagHorizTolFrac * hr) {
    mode_ = Mode::Gated;
    return false;
  }
  mode_ = isStopped ? Mode::Stopped : Mode::Spinning;
  sinceUpdateS_ = 0.0f;
  magOut = lp_;
  refOut = ref.field;
  return true;
}
