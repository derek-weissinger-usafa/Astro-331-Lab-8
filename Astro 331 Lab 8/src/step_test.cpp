#include "step_test.h"

#include <math.h>

#include "control_params.h"

using namespace params;

float StepTest::targetDeg(int i) const {
  switch (i) {
    case 1: return amp_;
    case 3: return -amp_;
    default: return 0.0f;  // baseline, back to 0
  }
}

float StepTest::offsetRad() const { return index_ < 0 ? 0.0f : targetDeg(index_) * 0.01745329f; }

void StepTest::start(float ampDeg) {
  amp_ = ampDeg;
  index_ = 0;
  sumIzz_ = 0.0f;
  nIzz_ = 0;
  t_ = -1.0f;  // segment begins on the first update
}

void StepTest::beginSegment(float yawDeg, float omegaZ, float hz) {
  t_ = 0.0f;
  y0_ = yawDeg;
  w0_ = omegaZ;
  h0_ = hz;
  riseLo_ = riseHi_ = -1.0f;
  peak_ = 0.0f;
  lastOut_ = 0.0f;
  tN_ = tSum_ = tSq_ = 0.0;
  sYY_ = sYQ_ = sQQ_ = sYX_ = sQX_ = xInt_ = 0.0;
}

StepTest::Event StepTest::update(float dt, float yawDeg, float omegaZ, float hz) {
  if (index_ < 0) return Event::None;
  if (t_ < 0.0f) beginSegment(yawDeg, omegaZ, hz);
  t_ += dt;

  const float target = targetDeg(index_);
  const float step = target - targetDeg(index_ - 1 < 0 ? 0 : index_ - 1);
  const float err = yawDeg - target;
  if (index_ > 0 && step != 0.0f) {
    const float frac = (yawDeg - y0_) / step;  // 0 at the start, 1 at the target
    if (riseLo_ < 0.0f && frac >= 0.1f) riseLo_ = t_;
    if (riseHi_ < 0.0f && frac >= 0.9f) riseHi_ = t_;
    if (frac - 1.0f > peak_) peak_ = frac - 1.0f;
    const float band = fmaxf(0.1f * fabsf(step), kStepBandMinDeg);
    if (fabsf(err) > band) lastOut_ = t_;
  }
  if (t_ <= kStepIdS) {
    xInt_ += (double)(hz - h0_) * dt;
    const double y = (yawDeg - y0_) * 0.0174532925, q = 0.5 * (double)t_ * t_;
    sYY_ += y * y;
    sYQ_ += y * q;
    sQQ_ += q * q;
    sYX_ += y * xInt_;
    sQX_ += q * xInt_;
  }
  if (t_ > kStepHoldS - kStepTailS) {
    tN_ += 1.0;
    tSum_ += err;
    tSq_ += (double)err * err;
  }
  if (t_ < kStepHoldS) return Event::None;

  Result r;
  r.targetDeg = step;
  if (index_ > 0) {
    r.riseS = (riseLo_ >= 0.0f && riseHi_ >= 0.0f) ? riseHi_ - riseLo_ : -1.0f;
    r.overshootPct = 100.0f * peak_;
    r.settleS = lastOut_ < kStepHoldS - kStepTailS ? lastOut_ : -1.0f;
    // Normal equations of  I y - tau q = -x :  [Syy -Syq; -Syq Sqq] [I; tau] = [-Syx; Sqx]
    const double det = sYY_ * sQQ_ - sYQ_ * sYQ_;
    if (fabsf(step) >= 1.0f && det > 1e-12 * sYY_ * sQQ_) {
      r.izz = (float)((-sYX_ * sQQ_ + sYQ_ * sQX_) / det);
      r.dragMNm = (float)((sYY_ * sQX_ - sYQ_ * sYX_) / det * 1e3);
      if (r.izz > 0.0f) {
        sumIzz_ += r.izz;
        nIzz_++;
      }
    }
  }
  const double mean = tN_ > 0 ? tSum_ / tN_ : 0.0;
  r.errDeg = (float)mean;
  r.jitterDeg = tN_ > 1 ? (float)sqrt(fmax(tSq_ / tN_ - mean * mean, 0.0)) : 0.0f;
  last_ = r;
  const Event ev = index_ == 0 ? Event::BaselineDone : Event::StepDone;
  if (++index_ > kSteps) {
    index_ = -1;
    return Event::Complete;
  }
  t_ = -1.0f;
  return ev;
}
