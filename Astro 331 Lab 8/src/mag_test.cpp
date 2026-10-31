#include "mag_test.h"

#include <math.h>

#include "control_params.h"

using namespace params;

const mag_test::Step& MagTest::step() const {
  const int i = index_ < 0 ? 0 : index_;
  if (i < mag_test::kNumMain) return mag_test::kMain[i];
  return mag_test::kDrift[(i - mag_test::kNumMain) % mag_test::kNumDrift];
}

void MagTest::start(int driftSteps) {
  driftSteps_ = driftSteps < 0 ? 0 : driftSteps;
  index_ = 0;
  passN_ = doneN_ = 0;
  driftT_ = driftMax_ = 0.0f;
  driftN_ = driftAid_ = 0;
  sT_ = sY_ = sTT_ = sTY_ = 0.0;
  beginStep();
}

void MagTest::beginStep() {
  t_ = 0.0f;
  n_ = aidN_ = 0;
  residSq_ = 0.0;
  maxAbs_ = peak_ = lastOutT_ = 0.0f;
}

MagTest::Event MagTest::update(float dt, float yawDeg, bool aidInUse, float residDeg) {
  if (index_ < 0) return Event::None;
  const mag_test::Step& s = step();
  const bool kick = s.kickDeg != 0;
  t_ += dt;
  yaw_ = yawDeg;
  const float a = fabsf(yawDeg);
  n_++;
  if (aidInUse) {
    aidN_++;
    residSq_ += (double)residDeg * residDeg;
  }
  if (a > peak_) peak_ = a;
  if (a > kMagTestTolDeg) lastOutT_ = t_;
  // KICK steps: max |yaw| over the rest of the step once the allowed recovery time is over.
  if (!kick || t_ > kMagTestRecoverS) maxAbs_ = fmaxf(maxAbs_, a);

  if (inDrift()) {
    driftT_ += dt;
    driftN_++;
    if (aidInUse) driftAid_++;
    if (a > driftMax_) driftMax_ = a;
    sT_ += driftT_;
    sY_ += yawDeg;
    sTT_ += (double)driftT_ * driftT_;
    sTY_ += (double)driftT_ * yawDeg;
  }

  if (t_ < s.s) return Event::None;

  Result r;
  r.kick = kick;
  r.peakDeg = peak_;
  r.endDeg = yaw_;
  r.aidFrac = n_ ? (float)aidN_ / n_ : 0.0f;
  r.residRmsDeg = aidN_ ? sqrtf((float)(residSq_ / aidN_)) : 0.0f;
  r.recoverS = lastOutT_;
  r.maxAbsDeg = maxAbs_;
  // KICK: never outside the limit after the recovery time (so also at the end)
  r.pass = kick ? lastOutT_ <= kMagTestRecoverS : maxAbs_ <= kMagTestTolDeg;
  last_ = r;
  doneN_++;
  if (r.pass) passN_++;

  if (++index_ >= total()) {
    index_ = -1;
    return Event::Complete;
  }
  beginStep();
  return Event::StepDone;
}

float MagTest::driftTrendDegPerH() const {
  if (driftN_ < 2) return 0.0f;
  const double n = driftN_;
  const double den = n * sTT_ - sT_ * sT_;
  if (den <= 0.0) return 0.0f;
  return (float)((n * sTY_ - sT_ * sY_) / den * 3600.0);
}
