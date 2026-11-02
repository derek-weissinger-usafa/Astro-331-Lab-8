#include "full_demo.h"

#include <math.h>

#include "control_params.h"

void FullDemo::start(float holdS, int faultWheel) {
  holdS_ = holdS;
  wheel_ = faultWheel;
  t_ = maxErr_ = 0.0f;
  n_ = sum_ = sq_ = 0;
  n0_ = sum0_ = n1_ = sum1_ = 0;
  phase_ = Phase::Hold;
}

bool FullDemo::holdStep(float dt, float yawErrDeg) {
  if (phase_ != Phase::Hold) return false;
  t_ += dt;
  if (fabsf(yawErrDeg) > maxErr_) maxErr_ = fabsf(yawErrDeg);
  n_ += 1;
  sum_ += yawErrDeg;
  sq_ += (double)yawErrDeg * yawErrDeg;
  if (t_ <= params::kFullDriftWinS) {
    n0_ += 1;
    sum0_ += yawErrDeg;
  }
  if (t_ > holdS_ - params::kFullDriftWinS) {
    n1_ += 1;
    sum1_ += yawErrDeg;
  }
  return t_ >= holdS_;
}

float FullDemo::holdJitterDeg() const {
  if (n_ < 2) return 0.0f;
  const double m = sum_ / n_;
  const double v = sq_ / n_ - m * m;
  return v > 0 ? (float)sqrt(v) : 0.0f;
}

float FullDemo::holdDriftDeg() const {
  if (n0_ < 1 || n1_ < 1) return 0.0f;
  return (float)(sum1_ / n1_ - sum0_ / n0_);
}

bool FullDemo::holdPass() const { return n_ > 0 && maxErr_ <= params::kFullHoldTolDeg; }

char FullDemo::phaseChar() const {
  switch (phase_) {
    case Phase::Hold: return '1';
    case Phase::Slew: return '2';
    case Phase::Fault: return '3';
    case Phase::Done: return 'D';
    default: return '-';
  }
}
