#include "slew_test.h"

#include "control_params.h"
#include "tune.h"
#include "wheel_geometry.h"

static constexpr float kPi = 3.14159265f;

static float wrapPi(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

// Cosine-profile duration for a rotation of ang rad: peak rate = (pi/2) ang / T, peak acceleration =
// (pi^2/2) ang / T^2, at least kSlewMinS.
static float profileTime(float ang, bool tilt = false) {
  const float rate = tilt ? params::kTiltSlewMaxRateRadS : params::kSlewMaxRateRadS;
  const float acc = tilt ? params::kTiltSlewMaxAccelRadS2 : params::kSlewMaxAccelRadS2;
  const float tRate = (kPi / 2.0f) * ang / rate;
  const float tAcc = kPi * sqrtf(ang / (2.0f * acc));
  float T = params::kSlewMinS;
  if (tRate > T) T = tRate;
  if (tAcc > T) T = tAcc;
  return T;
}

SlewTest::SlewTest() {
  for (int i = 0; i < kMaxTargets; i++) {
    targets_[i].rollDeg = params::kSlewTargets[i][0];
    targets_[i].pitchDeg = params::kSlewTargets[i][1];
    targets_[i].yawDeg = params::kSlewTargets[i][2];
    tiltTargets_[i].rollDeg = params::kSlewTiltTargets[i][0];
    tiltTargets_[i].pitchDeg = params::kSlewTiltTargets[i][1];
    tiltTargets_[i].yawDeg = params::kSlewTiltTargets[i][2];
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

bool SlewTest::setTiltTarget(int i, float dRollDeg, float dPitchDeg, float yawDeg) {
  if (i < 0 || i >= params::kSlewNumTargets) return false;
  if (fabsf(dRollDeg) > params::kTiltMaxOffsetDeg || fabsf(dPitchDeg) > params::kTiltMaxOffsetDeg) return false;
  tiltTargets_[i].rollDeg = dRollDeg;
  tiltTargets_[i].pitchDeg = dPitchDeg;
  tiltTargets_[i].yawDeg = yawDeg;
  return true;
}

// x/y momentum the wheels can take before the slew abort, less a margin: two wheels per tilt axis,
// each along it at cos(elevation).
float SlewTest::tiltCapacity() {
  return 2.0f * cosf(wheel_geometry::kElevationDeg * kDegToRad) * params::kWheelInertia *
         (params::kDemoWheelAbortRadS - params::kTiltCapMarginRadS);
}

// Momentum target i stores: k * |tilt offset| * dwell, dwell = half the slew in (the offset ramps
// up) + settle + hold. Unloading empties the wheels before each target, so targets don't add up.
float SlewTest::tiltBudget(int i, float k) const {
  const Target& g = tiltTargets_[i];
  const Target p = i > 0 ? tiltTargets_[i - 1] : Target();
  const float d = sqrtf(g.rollDeg * g.rollDeg + g.pitchDeg * g.pitchDeg) * kDegToRad;
  const float dr = (g.rollDeg - p.rollDeg) * kDegToRad, dp = (g.pitchDeg - p.pitchDeg) * kDegToRad;
  const float dy = wrapPi((g.yawDeg - p.yawDeg) * kDegToRad);
  const float dwell = 0.5f * profileTime(sqrtf(dr * dr + dp * dp + dy * dy), true) + params::kSlewSettleS + 1.0f +
                      params::kSlewHoldS;
  return k * d * dwell;
}

// Reference body rate that makes the damping term leak the wheels' x/y momentum (kMomentumLeakTauS).
static Vec3 momentumLeakRate(const Vec3& hWheels) {
  const float kdx = 2.0f * tune::attZeta * tune::inertia[0] * tune::attBw;
  const float kdy = 2.0f * tune::attZeta * tune::inertia[1] * tune::attBw;
  return {kdx > 0.0f ? hWheels.x / (kdx * params::kMomentumLeakTauS) : 0.0f,
          kdy > 0.0f ? hWheels.y / (kdy * params::kMomentumLeakTauS) : 0.0f, 0.0f};
}

void SlewTest::restRad(float psiRel, float& roll, float& pitch) const {
  const float c = cosf(psiRel), s = sinf(psiRel);
  roll = fitRoll_[0] + fitRoll_[1] * c + fitRoll_[2] * s;
  pitch = fitPitch_[0] + fitPitch_[1] * c + fitPitch_[2] * s;
}

void SlewTest::restAt(float psiDeg, float& rollDeg, float& pitchDeg) const {
  float r, p;
  restRad(psiDeg * kDegToRad, r, p);
  rollDeg = r * kRadToDeg;
  pitchDeg = p * kRadToDeg;
}

void SlewTest::solveRestFit() {
  Mat<3, 3> N, Ni;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) N(r, c) = nrm_[r][c];
  if (inverse3(N, Ni)) {
    for (int r = 0; r < 3; r++) {
      fitRoll_[r] = fitPitch_[r] = 0.0f;
      for (int c = 0; c < 3; c++) {
        fitRoll_[r] += Ni(r, c) * rhsRoll_[c];
        fitPitch_[r] += Ni(r, c) * rhsPitch_[c];
      }
    }
  } else if (nrm_[0][0] > 0.0f) {  // too little of a turn to fit the wave: the mean only
    fitRoll_[0] = rhsRoll_[0] / nrm_[0][0];
    fitPitch_[0] = rhsPitch_[0] / nrm_[0][0];
    fitRoll_[1] = fitRoll_[2] = fitPitch_[1] = fitPitch_[2] = 0.0f;
  }
  for (int r = 0; r < 3; r++) {
    fitRollDeg_[r] = fitRoll_[r] * kRadToDeg;
    fitPitchDeg_[r] = fitPitch_[r] * kRadToDeg;
  }
}

Quat SlewTest::targetQuat(int i) const {
  if (mode_ == Mode::YawOnly) return qFromEuler321(rollRest_, pitchRest_, yaw0_ + targets_[i].yawDeg * kDegToRad);
  if (mode_ == Mode::Tilt) {
    const Target& g = tiltTargets_[i];
    float r, p;
    restRad(g.yawDeg * kDegToRad, r, p);
    return qFromEuler321(r + g.rollDeg * kDegToRad, p + g.pitchDeg * kDegToRad, yaw0_ + g.yawDeg * kDegToRad);
  }
  return qFromEuler321(targets_[i].rollDeg * kDegToRad, targets_[i].pitchDeg * kDegToRad,
                       yaw0_ + targets_[i].yawDeg * kDegToRad);
}

void SlewTest::start(const Quat& qNow, Mode mode, float gravK) {
  mode_ = mode;
  gravK_ = gravK;
  const Vec3 e0 = qToEuler321(qNow);
  yaw0_ = e0.z;
  rollRest_ = e0.x;
  pitchRest_ = e0.y;
  idx_ = 0;
  elapsed_ = 0.0f;
  ff_ = {0.0f, 0.0f, 0.0f};
  lpRoll_ = e0.x;
  lpPitch_ = e0.y;
  for (int i = 0; i < kMaxTargets; i++) results_[i] = Result();
  if (mode_ == Mode::Tilt) {
    for (int r = 0; r < 3; r++) {
      rhsRoll_[r] = rhsPitch_[r] = fitRoll_[r] = fitPitch_[r] = 0.0f;
      for (int c = 0; c < 3; c++) nrm_[r][c] = 0.0f;
    }
    fitRoll_[0] = e0.x;  // until the scan is fitted
    fitPitch_[0] = e0.y;
    lpRoll_ = e0.x;
    lpPitch_ = e0.y;
    T_ = (kPi / 2.0f) * 2.0f * kPi / params::kTiltScanRateRadS;  // cosine profile, peak kTiltScanRateRadS
    phi_ = {0.0f, 0.0f, 2.0f * kPi};
    t_ = 0.0f;
    phase_ = Phase::Scan;
    setYawRef(e0.x, e0.y, yaw0_, 0.0f, 0.0f);
    return;
  }
  beginSlew(qnormalize(qNow));
}

void SlewTest::beginSlew(const Quat& qFrom) {
  qFrom_ = qFrom;
  qTarget_ = targetQuat(idx_);
  const Vec3 et = qToEuler321(qTarget_);
  tgtRoll_ = et.x;
  tgtPitch_ = et.y;
  if (mode_ == Mode::YawOnly) {
    // Turn about the vertical by the shorter way round.
    yawFrom_ = qToEuler321(qFrom_).z;
    dYaw_ = wrapPi(yaw0_ + targets_[idx_].yawDeg * kDegToRad - yawFrom_);
    phi_ = {0.0f, 0.0f, dYaw_};
  } else {
    phi_ = qToRotVec(qmul(qTarget_, qconj(qFrom_)));  // shortest rotation (qToRotVec takes w >= 0)
  }
  T_ = profileTime(norm(phi_), mode_ == Mode::Tilt);
  t_ = 0.0f;
  okT_ = 0.0f;
  phase_ = Phase::Slew;
  refQ_ = qFrom_;
  refW_ = refA_ = {0.0f, 0.0f, 0.0f};
}

// Unload at the current reference heading, starting from the current reference tilt offset so the
// reference doesn't jump.
void SlewTest::beginUnload() {
  const Vec3 er = qToEuler321(refQ_);
  unloadYaw_ = er.z;
  float r0, p0;
  restRad(wrapPi(er.z - yaw0_), r0, p0);
  delta_ = {er.x - r0, er.y - p0, 0.0f};
  t_ = 0.0f;
  okT_ = 0.0f;
  hUnload0_ = -1.0f;
  phase_ = Phase::Unload;
}

void SlewTest::setYawRef(float roll, float pitch, float yaw, float yawRate, float yawAcc) {
  refQ_ = qFromEuler321(roll, pitch, yaw);
  // 3-2-1 kinematics with no roll/pitch rate: p = -psi' sin(th), q = psi' cos(th) sin(ph),
  // r = psi' cos(th) cos(ph) (level: (0, 0, psi'), as DEMO SPIN's spin reference).
  const Vec3 axis = {-sinf(pitch), cosf(pitch) * sinf(roll), cosf(pitch) * cosf(roll)};
  refW_ = axis * yawRate;
  refA_ = axis * yawAcc;
}

void SlewTest::updateError(const Quat& qEst) {
  const Vec3 e = qToEuler321(qEst);
  if (mode_ == Mode::YawOnly) {
    const Target& g = targets_[idx_];
    errDeg_ = {wrapPi(e.x - rollRest_) * kRadToDeg, wrapPi(e.y - pitchRest_) * kRadToDeg,
               wrapPi(e.z - (yaw0_ + g.yawDeg * kDegToRad)) * kRadToDeg};
    return;
  }
  const bool tl = mode_ == Mode::Tilt;
  const float yawT = yaw0_ + (tl ? tiltTargets_[idx_].yawDeg : targets_[idx_].yawDeg) * kDegToRad;
  const float rollT = tl ? tgtRoll_ : targets_[idx_].rollDeg * kDegToRad;
  const float pitchT = tl ? tgtPitch_ : targets_[idx_].pitchDeg * kDegToRad;
  errDeg_ = {wrapPi(e.x - rollT) * kRadToDeg, wrapPi(e.y - pitchT) * kRadToDeg, wrapPi(e.z - yawT) * kRadToDeg};
}

SlewTest::Event SlewTest::step(float dt, const Quat& qEst, const Vec3& hWheels) {
  if (phase_ == Phase::Off) return Event::None;
  t_ += dt;
  elapsed_ += dt;
  const Vec3 eNow = qToEuler321(qEst);
  if (mode_ == Mode::YawOnly) {
    float a = dt / params::kDemoRestAvgS;
    if (a > 1.0f) a = 1.0f;
    rollRest_ += (eNow.x - rollRest_) * a;
    pitchRest_ += (eNow.y - pitchRest_) * a;
  }
  updateError(qEst);
  const Event ev = advance(dt, qEst, hWheels);
  if (mode_ == Mode::YawOnly && phase_ != Phase::Off) {
    // Roll/pitch reference = measured tilt (damping only); yaw from the profile or the target.
    // As in the SLEW TILT scan, the reference rate includes the slow part of the tilt rate, so the
    // damping doesn't fight the rest tilt drifting as the table turns (a room-fixed torque / k: at a
    // 3 mm CG, 1.5 mN m filled a wheel in simulation without this).
    float lpa = dt / params::kTiltScanTrackTauS;
    if (lpa > 1.0f) lpa = 1.0f;
    lpRoll_ += (eNow.x - lpRoll_) * lpa;
    lpPitch_ += (eNow.y - lpPitch_) * lpa;
    if (phase_ == Phase::Slew) {
      const float x = kPi * t_ / T_;
      const float s = 0.5f * (1.0f - cosf(x));
      const float sd = (kPi / (2.0f * T_)) * sinf(x);
      const float sdd = (kPi * kPi / (2.0f * T_ * T_)) * cosf(x);
      setYawRef(eNow.x, eNow.y, yawFrom_ + s * dYaw_, sd * dYaw_, sdd * dYaw_);
    } else {
      setYawRef(eNow.x, eNow.y, yaw0_ + targets_[idx_].yawDeg * kDegToRad, 0.0f, 0.0f);
    }
    refW_.x += (eNow.x - lpRoll_) / params::kTiltScanTrackTauS;
    refW_.y += (eNow.y - lpPitch_) / params::kTiltScanTrackTauS;
    const Vec3 leak = momentumLeakRate(hWheels);
    refW_.x += leak.x;
    refW_.y += leak.y;
  }
  if (mode_ == Mode::Tilt) {
    if (phase_ == Phase::Scan || phase_ == Phase::Off || gravK_ <= 0.0f) {
      ff_ = {0.0f, 0.0f, 0.0f};
    } else {
      // Gravity holding torque for the reference tilt away from the rest tilt at its heading.
      const Vec3 er = qToEuler321(refQ_);
      float r0, p0;
      restRad(wrapPi(er.z - yaw0_), r0, p0);
      ff_ = {gravK_ * wrapPi(er.x - r0), gravK_ * wrapPi(er.y - p0), 0.0f};
    }
  }
  return ev;
}

SlewTest::Event SlewTest::advance(float dt, const Quat& qEst, const Vec3& hWheels) {
  const float tol = params::kSlewTolDeg;
  const bool within = fabsf(errDeg_.x) < tol && fabsf(errDeg_.y) < tol && fabsf(errDeg_.z) < tol;

  switch (phase_) {
    case Phase::Scan: {
      const Vec3 e = qToEuler321(qEst);
      if (t_ >= T_) {
        solveRestFit();
        setYawRef(e.x, e.y, yaw0_, 0.0f, 0.0f);  // back at the start heading (+360)
        beginUnload();
        return Event::ScanDone;
      }
      const float x = kPi * t_ / T_;
      const float s = 0.5f * (1.0f - cosf(x));
      const float sd = (kPi / (2.0f * T_)) * sinf(x);
      const float sdd = (kPi * kPi / (2.0f * T_ * T_)) * cosf(x);
      // Roll/pitch reference = the measured tilt (no stiffness), and the reference rate includes the
      // SLOW part of the tilt rate (derivative of a kTiltScanTrackTauS low-pass), so the damping acts
      // only on faster swinging, not on the rest tilt drifting as the table turns (kTiltScanTrackTauS).
      // Small angles: roll rate ~ body x, pitch rate ~ body y.
      float a = dt / params::kTiltScanTrackTauS;
      if (a > 1.0f) a = 1.0f;
      lpRoll_ += (e.x - lpRoll_) * a;
      lpPitch_ += (e.y - lpPitch_) * a;
      setYawRef(e.x, e.y, yaw0_ + s * 2.0f * kPi, sd * 2.0f * kPi, sdd * 2.0f * kPi);
      refW_.x += (e.x - lpRoll_) / params::kTiltScanTrackTauS;
      refW_.y += (e.y - lpPitch_) / params::kTiltScanTrackTauS;
      const Vec3 leak = momentumLeakRate(hWheels);
      refW_.x += leak.x;
      refW_.y += leak.y;
      if (t_ > params::kTiltScanSkipS) {
        const float psi = e.z - yaw0_;
        const float v[3] = {1.0f, cosf(psi), sinf(psi)};
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 3; c++) nrm_[r][c] += v[r] * v[c];
          rhsRoll_[r] += v[r] * e.x;
          rhsPitch_[r] += v[r] * e.y;
        }
      }
      return Event::None;
    }
    case Phase::Unload: {
      const float hxy = sqrtf(hWheels.x * hWheels.x + hWheels.y * hWheels.y);
      if (hUnload0_ < 0.0f) hUnload0_ = hxy;
      // Tilt offset that drains the stored x/y momentum with time constant kUnloadTauS (holding the
      // table tilted by delta costs the wheels -k*delta; see the header), capped and rate-limited.
      Vec3 want = {0.0f, 0.0f, 0.0f};
      if (gravK_ > 0.0f)
        want = {hWheels.x / (gravK_ * params::kUnloadTauS), hWheels.y / (gravK_ * params::kUnloadTauS), 0.0f};
      const float cap = params::kUnloadMaxDeg * kDegToRad;
      const float n = norm(want);
      if (n > cap) want = want * (cap / n);
      const float stp = params::kUnloadRateDegS * kDegToRad * dt;
      delta_.x += fmaxf(-stp, fminf(stp, want.x - delta_.x));
      delta_.y += fmaxf(-stp, fminf(stp, want.y - delta_.y));
      float r0, p0;
      restRad(wrapPi(unloadYaw_ - yaw0_), r0, p0);
      refQ_ = qFromEuler321(r0 + delta_.x, p0 + delta_.y, unloadYaw_);
      refW_ = refA_ = {0.0f, 0.0f, 0.0f};
      okT_ = hxy < params::kUnloadDoneFrac * tiltCapacity() ? okT_ + dt : 0.0f;
      if (okT_ >= params::kUnloadDoneS || t_ >= params::kUnloadTimeoutS) {
        lastUnloadS_ = t_;
        beginSlew(refQ_);
        return Event::UnloadDone;
      }
      return Event::None;
    }
    case Phase::Slew: {
      if (t_ >= T_) {
        refQ_ = qTarget_;
        refW_ = refA_ = {0.0f, 0.0f, 0.0f};
        phase_ = Phase::Settle;
        t_ = 0.0f;
        okT_ = 0.0f;
        return Event::SlewDone;
      }
      if (mode_ == Mode::YawOnly) return Event::None;  // step() sets the yaw-only reference
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
        if (mode_ == Mode::Tilt) {
          restAt(tiltTargets_[idx_].yawDeg, r.restRollDeg, r.restPitchDeg);
        } else {
          r.restRollDeg = rollRest_ * kRadToDeg;
          r.restPitchDeg = pitchRest_ * kRadToDeg;
        }
        r.pass = r.settled && r.maxErrDeg.x <= tol && r.maxErrDeg.y <= tol && r.maxErrDeg.z <= tol;
        if (idx_ + 1 >= params::kSlewNumTargets) {
          phase_ = Phase::Done;  // results final; keep holding the last target until STOP
          return Event::Complete;
        }
        idx_++;
        if (mode_ == Mode::Tilt) {
          beginUnload();
        } else {
          beginSlew(qTarget_);  // continue from the reference, not the estimate: no step in q_ref
        }
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
    case Phase::Scan: return 'C';
    case Phase::Unload: return 'U';
    case Phase::Slew: return 'S';
    case Phase::Settle: return 'W';
    case Phase::Hold: return 'H';
    case Phase::Done: return 'D';
    default: return '-';
  }
}
