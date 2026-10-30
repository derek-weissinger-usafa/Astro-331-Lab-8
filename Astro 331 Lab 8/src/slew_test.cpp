#include "slew_test.h"

#include "control_params.h"

static constexpr float kPi = 3.14159265f;

static float wrapPi(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

SlewTest::SlewTest() {
  for (int i = 0; i < kMaxTargets; i++) {
    targets_[i].rollDeg = params::kSlewTargets[i][0];
    targets_[i].pitchDeg = params::kSlewTargets[i][1];
    targets_[i].yawDeg = params::kSlewTargets[i][2];
  }
}

bool SlewTest::setTarget(int i, float rollDeg, float pitchDeg, float yawDeg) {
  if (i < 0 || i >= params::kSlewNumTargets) return false;
  // Tilt of body z from vertical for this roll/pitch: cos(tilt) = cos(roll) cos(pitch).
  const float c = cosf(rollDeg * kDegToRad) * cosf(pitchDeg * kDegToRad);
  const float tiltDeg = acosf(c > 1.0f ? 1.0f : c) * kRadToDeg;
  if (tiltDeg > params::kSlewMaxTiltDeg || fabsf(rollDeg) > params::kSlewMaxTiltDeg ||
      fabsf(pitchDeg) > params::kSlewMaxTiltDeg)
    return false;
  targets_[i].rollDeg = rollDeg;
  targets_[i].pitchDeg = pitchDeg;
  targets_[i].yawDeg = yawDeg;
  return true;
}

Quat SlewTest::targetQuat(int i) const {
  return qFromEuler321(targets_[i].rollDeg * kDegToRad, targets_[i].pitchDeg * kDegToRad,
                       yaw0_ + targets_[i].yawDeg * kDegToRad);
}

void SlewTest::start(const Quat& qNow) {
  yaw0_ = qToEuler321(qNow).z;
  idx_ = 0;
  elapsed_ = 0.0f;
  for (int i = 0; i < kMaxTargets; i++) results_[i] = Result();
  beginSlew(qnormalize(qNow));
}

void SlewTest::beginSlew(const Quat& qFrom) {
  qFrom_ = qFrom;
  qTarget_ = targetQuat(idx_);
  phi_ = qToRotVec(qmul(qTarget_, qconj(qFrom_)));  // shortest rotation (qToRotVec takes w >= 0)
  const float ang = norm(phi_);
  // Cosine profile: peak rate = (pi/2) ang / T, peak acceleration = (pi^2/2) ang / T^2.
  const float tRate = (kPi / 2.0f) * ang / params::kSlewMaxRateRadS;
  const float tAcc = kPi * sqrtf(ang / (2.0f * params::kSlewMaxAccelRadS2));
  T_ = params::kSlewMinS;
  if (tRate > T_) T_ = tRate;
  if (tAcc > T_) T_ = tAcc;
  t_ = 0.0f;
  okT_ = 0.0f;
  phase_ = Phase::Slew;
  refQ_ = qFrom_;
  refW_ = refA_ = {0.0f, 0.0f, 0.0f};
}

void SlewTest::updateError(const Quat& qEst) {
  const Vec3 e = qToEuler321(qEst);
  const Target& g = targets_[idx_];
  errDeg_ = {wrapPi(e.x - g.rollDeg * kDegToRad) * kRadToDeg, wrapPi(e.y - g.pitchDeg * kDegToRad) * kRadToDeg,
             wrapPi(e.z - (yaw0_ + g.yawDeg * kDegToRad)) * kRadToDeg};
}

SlewTest::Event SlewTest::step(float dt, const Quat& qEst) {
  if (phase_ == Phase::Off) return Event::None;
  t_ += dt;
  elapsed_ += dt;
  updateError(qEst);
  const float tol = params::kSlewTolDeg;
  const bool within = fabsf(errDeg_.x) < tol && fabsf(errDeg_.y) < tol && fabsf(errDeg_.z) < tol;

  switch (phase_) {
    case Phase::Slew: {
      if (t_ >= T_) {
        refQ_ = qTarget_;
        refW_ = refA_ = {0.0f, 0.0f, 0.0f};
        phase_ = Phase::Settle;
        t_ = 0.0f;
        okT_ = 0.0f;
        return Event::SlewDone;
      }
      // q_ref = exp(s phi) (x) q_from with q_dot = -1/2 [0,w] (x) q  =>  w = -s_dot phi (see demo_spin.cpp).
      const float x = kPi * t_ / T_;
      const float s = 0.5f * (1.0f - cosf(x));
      const float sd = (kPi / (2.0f * T_)) * sinf(x);
      const float sdd = (kPi * kPi / (2.0f * T_ * T_)) * cosf(x);
      refQ_ = qnormalize(qmul(qFromRotVec(phi_ * s), qFrom_));
      refW_ = phi_ * (-sd);
      refA_ = phi_ * (-sdd);
      return Event::None;
    }
    case Phase::Settle: {
      okT_ = within ? okT_ + dt : 0.0f;
      if (okT_ >= params::kSlewSettleS) {
        results_[idx_].settled = true;
        results_[idx_].settleS = t_;
        phase_ = Phase::Hold;
        t_ = 0.0f;
        return Event::Settled;
      }
      if (t_ >= params::kSlewSettleTimeoutS) {
        results_[idx_].settled = false;
        results_[idx_].settleS = t_;
        phase_ = Phase::Hold;
        t_ = 0.0f;
        return Event::SettleTimeout;
      }
      return Event::None;
    }
    case Phase::Hold: {
      Result& r = results_[idx_];
      if (fabsf(errDeg_.x) > r.maxErrDeg.x) r.maxErrDeg.x = fabsf(errDeg_.x);
      if (fabsf(errDeg_.y) > r.maxErrDeg.y) r.maxErrDeg.y = fabsf(errDeg_.y);
      if (fabsf(errDeg_.z) > r.maxErrDeg.z) r.maxErrDeg.z = fabsf(errDeg_.z);
      if (t_ >= params::kSlewHoldS) {
        r.pass = r.settled && r.maxErrDeg.x <= tol && r.maxErrDeg.y <= tol && r.maxErrDeg.z <= tol;
        if (idx_ + 1 >= params::kSlewNumTargets) {
          phase_ = Phase::Done;  // results final; keep holding the last target until STOP
          return Event::Complete;
        }
        idx_++;
        beginSlew(qTarget_);  // continue from the reference, not the estimate: no step in q_ref
        return Event::TargetDone;
      }
      return Event::None;
    }
    default:
      return Event::None;
  }
}

int SlewTest::passCount() const {
  int n = 0;
  for (int i = 0; i < params::kSlewNumTargets; i++) n += results_[i].pass ? 1 : 0;
  return n;
}

char SlewTest::phaseChar() const {
  switch (phase_) {
    case Phase::Slew: return 'S';
    case Phase::Settle: return 'W';
    case Phase::Hold: return 'H';
    case Phase::Done: return 'D';
    default: return '-';
  }
}
