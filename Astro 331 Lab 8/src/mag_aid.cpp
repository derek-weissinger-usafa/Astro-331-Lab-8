#include "mag_aid.h"

#include <math.h>

#include "control_params.h"

using namespace params;

// Mean field of each spinning wheel vs its speed, fixed table. Per wheel and body axis:
//   field = const + kDirStep * sign(speed) + kSpeedSlope * (|speed| - kMagRefSpeedRadS)
// The constant is absorbed by the learned reference. Values from MAG MAP 2026-10-30
// (test_runs/MAGMAP_0003.CSV, analysis MAGMAP_0003_analysis.html: each wheel alone at +-100/200
// rpm, 14 s settled means, spare Teensy, all four wheels). A-C agree with the 2026-10-27 sweep
// (test 3: 0.319/0.332/-0.082, -0.471/0.109/-0.007, -0.717/-0.232/-0.005 uT) within ~0.15 uT;
// D is measured for the first time (its encoder was dead before). The direction step is the big
// term (0.3-1.0 uT, 0.7-1.7 deg of heading per reversal); speed hardly matters once spinning
// (<= 0.011 uT per rad/s). All four reversing together matches the sum of the single-wheel steps
// within 0.2-0.45 deg. Since each direction pattern has its own reference, kDirStep only matters
// within a pattern (where it is constant); it is kept as the measured record.
static const float kSpeedSlope[4][3] = {{-0.0112f, 0.0037f, 0.0035f},
                                        {-0.0074f, -0.0036f, 0.0005f},
                                        {0.0056f, -0.0005f, 0.0041f},
                                        {0.0009f, -0.0030f, -0.0016f}};
static const float kDirStep[4][3] = {{0.304f, 0.411f, -0.039f},
                                     {-0.555f, 0.128f, -0.033f},
                                     {-0.577f, -0.353f, -0.036f},
                                     {0.287f, -0.988f, 0.004f}};

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

// Known spinning reference whose direction pattern differs from `pattern` in the fewest enabled
// wheels (-1 if none).
int MagAid::nearestSpinRef(int pattern, const bool enabled[4]) const {
  int best = -1, bestD = 99;
  for (int k = 0; k < kSpinRefs; k++) {
    if (!spin_[k].valid || k == pattern) continue;
    int d = 0;
    for (int i = 0; i < 4; i++)
      if (enabled[i] && (((k ^ pattern) >> i) & 1)) d++;
    if (d < bestD) {
      bestD = d;
      best = k;
    }
  }
  return best;
}

void MagAid::setHardIron(const Vec3& h) {
  if (h.x == hard_.x && h.y == hard_.y && h.z == hard_.z) return;
  hard_ = h;
  invalidateRefs();
}

void MagAid::invalidateRefs() {
  stopped_.valid = false;
  for (int k = 0; k < kSpinRefs; k++) spin_[k].valid = false;
  lpInit_ = false;
  settleS_ = 0.0f;  // restart the settle; learning is allowed if the aid was fresh
  learnS_ = 0.0f;
  learnN_ = 0;
  learnSum_ = {0, 0, 0};
}

void MagAid::setRoomCorrection(const float coefDeg[4]) {
  bool same = true;
  for (int k = 0; k < 4; k++) same = same && room_[k] == coefDeg[k] * kDegToRad;
  if (same) return;
  for (int k = 0; k < 4; k++) room_[k] = coefDeg[k] * kDegToRad;
  invalidateRefs();
}

Vec3 MagAid::roomCorrect(const Vec3& c) const {
  if (room_[0] == 0.0f && room_[1] == 0.0f && room_[2] == 0.0f && room_[3] == 0.0f) return c;
  const float th = atan2f(c.y, c.x);
  const float e = room_[0] * cosf(th) + room_[1] * sinf(th) + room_[2] * cosf(2.0f * th) + room_[3] * sinf(2.0f * th);
  const float ce = cosf(e), se = sinf(e);
  return {c.x * ce + c.y * se, -c.x * se + c.y * ce, c.z};  // rotate the horizontal part by -e
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
  const Vec3 c = roomCorrect(correct(magRaw - hard_, speed, enabled));
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

  if (!match && !isStopped && (predictAlways_ || (learnN_ == 0 && sinceUpdateS_ > kMagLearnMaxGapS))) {
    // New direction pattern that can't be learned now: predict it from the closest known one.
    const int k = nearestSpinRef(pattern & (kSpinRefs - 1), enabled);
    if (k >= 0) {
      ref.field = spin_[k].field;
      ref.valid = true;
      predicted_++;
      match = true;
    }
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
