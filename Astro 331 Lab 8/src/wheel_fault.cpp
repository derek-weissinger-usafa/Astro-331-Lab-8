#include "wheel_fault.h"

#include "control_params.h"

static float absMax(float a, float b) { return fabsf(b) > a ? fabsf(b) : a; }
static Vec3 absMax(const Vec3& m, const Vec3& e) { return {absMax(m.x, e.x), absMax(m.y, e.y), absMax(m.z, e.z)}; }
static bool within(const Vec3& e, float tol) { return fabsf(e.x) < tol && fabsf(e.y) < tol && fabsf(e.z) < tol; }

// ---- DeadMotorDetector ----
void DeadMotorDetector::reset() {
  blockT_ = effortSum_ = badS_ = 0.0f;
  haveSpeed0_ = false;
  dead_ = false;
}

bool DeadMotorDetector::update(float effort, float speed, float dt) {
  if (dead_) return false;
  if (!haveSpeed0_) {
    speed0_ = speed;
    haveSpeed0_ = true;
    blockT_ = effortSum_ = 0.0f;
    return false;
  }
  blockT_ += dt;
  effortSum_ += effort * dt;
  if (blockT_ < params::kDeadBlockS) return false;

  const float meanEffort = effortSum_ / blockT_;
  if (fabsf(meanEffort) >= params::kDeadEffort) {
    const float predicted = params::kMotorAccelPerEffort * effortSum_;  // speed gain (rad/s)
    const float gained = (speed - speed0_) * (meanEffort > 0.0f ? 1.0f : -1.0f);
    if (gained < params::kDeadResponseFrac * fabsf(predicted)) {
      badS_ += blockT_;
    } else {
      badS_ = 0.0f;
    }
  }
  speed0_ = speed;
  blockT_ = effortSum_ = 0.0f;
  if (badS_ >= params::kDeadDetectS - 1e-4f) {
    dead_ = true;
    return true;
  }
  return false;
}

// ---- FaultTest ----
void FaultTest::start(int wheel, float delayS) {
  real_ = wheel < 0;
  wheel_ = wheel;
  delay_ = delayS < 0.0f ? 0.0f : delayS;
  t_ = okT_ = holdT_ = 0.0f;
  detected_ = pass_ = false;
  detectS_ = recoverS_ = 0.0f;
  peak_ = holdMax_ = {0.0f, 0.0f, 0.0f};
  why_ = "";
  biasRadS_ = params::kFaultWheelBiasRadS;
  phase_ = real_ ? Phase::Watch : Phase::Wait;
}

FaultTest::Event FaultTest::finish(bool pass, const char* why) {
  pass_ = pass;
  why_ = why;
  phase_ = Phase::Done;
  return pass ? Event::Pass : Event::Fail;
}

FaultTest::Event FaultTest::onFailure(int wheel) {
  if (phase_ == Phase::Wait) return finish(false, "another wheel failed before the injection");
  if (phase_ != Phase::Watch || detected_) return Event::None;
  detected_ = true;
  if (real_) {
    wheel_ = wheel;
    t_ = 0.0f;  // REAL: recovery is timed from the detection
    peak_ = {0.0f, 0.0f, 0.0f};
    return Event::None;
  }
  detectS_ = t_;
  if (wheel != wheel_) return finish(false, "wrong wheel detected");
  return Event::None;
}

FaultTest::Event FaultTest::step(float dt, const Vec3& errDeg, float wheelSpeed) {
  const float tol = params::kFaultTolDeg;
  switch (phase_) {
    case Phase::Wait:
      t_ += dt;
      if (t_ >= delay_ && fabsf(wheelSpeed) >= params::kFaultMinWheelRadS) {
        phase_ = Phase::Watch;
        t_ = 0.0f;
        return Event::Inject;
      }
      return Event::None;
    case Phase::Watch:
      t_ += dt;
      if (real_ && !detected_) return Event::None;  // waiting for you to fail a wheel
      peak_ = absMax(peak_, errDeg);
      if (!real_ && !detected_ && t_ > params::kFaultRecoverMaxS) return finish(false, "not detected");
      if (detected_) {
        okT_ = within(errDeg, tol) ? okT_ + dt : 0.0f;
        if (okT_ >= params::kFaultSettleS) {
          recoverS_ = t_ - okT_;
          if (!real_ && recoverS_ < detectS_) recoverS_ = detectS_;
          phase_ = Phase::Hold;
          holdT_ = 0.0f;
          return Event::Recovered;
        }
      }
      if (t_ > params::kFaultRecoverMaxS + params::kFaultSettleS)
        return finish(false, "attitude not back within tolerance in time");
      return Event::None;
    case Phase::Hold:
      holdT_ += dt;
      holdMax_ = absMax(holdMax_, errDeg);
      if (holdT_ >= params::kFaultHoldS) {
        if (!real_ && detectS_ > params::kFaultDetectMaxS) return finish(false, "detected too late");
        if (recoverS_ > params::kFaultRecoverMaxS) return finish(false, "recovered too late");
        if (!within(holdMax_, tol + 1e-6f)) return finish(false, "error left tolerance during the hold");
        return finish(true, "");
      }
      return Event::None;
    default:
      return Event::None;
  }
}

char FaultTest::phaseChar() const {
  switch (phase_) {
    case Phase::Wait: return 'w';
    case Phase::Watch: return 'F';
    case Phase::Hold: return 'h';
    case Phase::Done: return pass_ ? 'P' : 'X';
    default: return '-';
  }
}
