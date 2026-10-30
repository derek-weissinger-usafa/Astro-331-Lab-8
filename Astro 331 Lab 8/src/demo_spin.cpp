#include "demo_spin.h"

#include "control_params.h"

static constexpr float kPi = 3.14159265f;

// Level attitude with yaw psi (inertial -> body): conj of an active rotation by psi about z.
static Quat levelQuat(float psi) { return qconj(qFromRotVec({0.0f, 0.0f, psi})); }

void DemoSpin::start(const Quat& qStart, float targetRate, bool levelOnly) {
  levelOnly_ = levelOnly;
  target_ = levelOnly ? 0.0f : targetRate;
  elapsed_ = 0.0f;
  rate_ = acc_ = 0.0f;
  t_ = 0.0f;
  levelOkT_ = 0.0f;
  yaw_ = qToEuler321(qStart).z;  // keep the current heading; only remove tilt

  if (levelOnly) {
    // DEMO LEVEL: start the rest-tilt reference at the current tilt (no slew to level).
    const Vec3 e = qToEuler321(qStart);
    rollRef_ = rollRest_ = e.x;
    pitchRef_ = pitchRest_ = e.y;
    phase_ = DemoPhase::Level;
    setRestRef();
    return;
  }

  // Slew q_ref(s) = exp(s*phi) (x) qStart, s: 0 -> 1, from the start attitude to level.
  qStart_ = qnormalize(qStart);
  phi_ = qToRotVec(qmul(levelQuat(yaw_), qconj(qStart_)));
  const float slew = (kPi / 2.0f) * norm(phi_) / params::kDemoLevelRateRadS;  // cosine peak rate
  levelT_ = slew > params::kDemoLevelMinS ? slew : params::kDemoLevelMinS;

  phase_ = DemoPhase::Level;
  setLevelRef(0.0f, 0.0f, 0.0f);
}

// With q = exp(s*phi) (x) qStart and q_dot = -1/2 [0,w] (x) q (frames.h), the reference body
// rate is w = -s_dot * phi (phi is a fixed body-frame axis, so the same in the reference frame).
void DemoSpin::setLevelRef(float s, float sdot, float sddot) {
  ref_.q = qnormalize(qmul(qFromRotVec(phi_ * s), qStart_));
  ref_.w = phi_ * (-sdot);
  ref_.a = phi_ * (-sddot);
}

void DemoSpin::setRestRef() {
  ref_.q = qFromEuler321(rollRef_, pitchRef_, yaw_);
  ref_.w = {0.0f, 0.0f, 0.0f};
  ref_.a = {0.0f, 0.0f, 0.0f};
}

// Level, rotating about +z at rate_: w = (0, 0, rate) in the reference body frame.
void DemoSpin::setSpinRef() {
  ref_.q = levelQuat(yaw_);
  ref_.w = {0.0f, 0.0f, rate_};
  ref_.a = {0.0f, 0.0f, acc_};
}

DemoSpin::Event DemoSpin::step(float dt, float roll, float pitch) {
  const float T = params::kDemoRampS;
  t_ += dt;
  elapsed_ += dt;
  if (levelOnly_ && (phase_ == DemoPhase::Level || phase_ == DemoPhase::LevelHold)) {
    // Reference tilt = measured tilt: no roll/pitch stiffness, only damping; heading stays held.
    rollRef_ = roll;
    pitchRef_ = pitch;
    setRestRef();
    float a = dt / params::kDemoRestAvgS;
    if (a > 1.0f) a = 1.0f;
    rollRest_ += (roll - rollRest_) * a;
    pitchRest_ += (pitch - pitchRest_) * a;
    if (phase_ == DemoPhase::Level) {
      const float tol = params::kDemoRestTolDeg * kDegToRad;
      if (fabsf(roll - rollRest_) < tol && fabsf(pitch - pitchRest_) < tol) {
        levelOkT_ += dt;
      } else {
        levelOkT_ = 0.0f;
      }
      if (levelOkT_ >= params::kDemoLevelHoldS) {
        phase_ = DemoPhase::LevelHold;
        t_ = 0.0f;
        return Event::Leveled;
      }
    }
    return Event::None;
  }
  switch (phase_) {
    case DemoPhase::Level: {
      if (t_ < levelT_) {
        const float x = kPi * t_ / levelT_;
        setLevelRef((1.0f - cosf(x)) * 0.5f, (kPi / (2.0f * levelT_)) * sinf(x),
                    (kPi * kPi / (2.0f * levelT_ * levelT_)) * cosf(x));
      } else {
        setLevelRef(1.0f, 0.0f, 0.0f);
      }
      const float tol = params::kDemoLevelTolDeg * kDegToRad;
      if (t_ >= levelT_ && fabsf(roll) < tol && fabsf(pitch) < tol) {
        levelOkT_ += dt;
      } else {
        levelOkT_ = 0.0f;
      }
      if (levelOkT_ >= params::kDemoLevelHoldS && levelOnly_) {
        phase_ = DemoPhase::LevelHold;  // keep the same level reference
        t_ = 0.0f;
        return Event::Leveled;
      }
      if (levelOkT_ >= params::kDemoLevelHoldS) {
        phase_ = DemoPhase::SpinUp;
        t_ = 0.0f;
        rate_ = acc_ = 0.0f;
        setSpinRef();
        return Event::Leveled;
      }
      if (!levelOnly_ && t_ > levelT_ + params::kDemoLevelTimeoutS) return Event::LevelTimeout;
      return Event::None;
    }
    case DemoPhase::LevelHold:
      setLevelRef(1.0f, 0.0f, 0.0f);  // hold level at the start heading until stopped
      return Event::None;
    case DemoPhase::SpinUp: {
      Event ev = Event::None;
      if (t_ >= T) {
        rate_ = target_;
        acc_ = 0.0f;
        phase_ = DemoPhase::Hold;
        t_ = 0.0f;
        ev = Event::HoldStarted;
      } else {
        const float x = kPi * t_ / T;
        rate_ = target_ * (1.0f - cosf(x)) * 0.5f;
        acc_ = target_ * (kPi / (2.0f * T)) * sinf(x);
      }
      yaw_ += rate_ * dt;
      setSpinRef();
      return ev;
    }
    case DemoPhase::Hold: {
      rate_ = target_;
      acc_ = 0.0f;
      yaw_ += rate_ * dt;
      setSpinRef();
      if (t_ >= params::kDemoHoldS) {
        downFrom_ = rate_;
        phase_ = DemoPhase::SpinDown;
        t_ = 0.0f;
        return Event::SpinDownStarted;
      }
      return Event::None;
    }
    case DemoPhase::SpinDown: {
      Event ev = Event::None;
      if (t_ >= T) {
        rate_ = acc_ = 0.0f;
        phase_ = DemoPhase::Settle;
        t_ = 0.0f;
        ev = Event::Settling;
      } else {
        const float x = kPi * t_ / T;
        rate_ = downFrom_ * (1.0f + cosf(x)) * 0.5f;
        acc_ = -downFrom_ * (kPi / (2.0f * T)) * sinf(x);
      }
      yaw_ += rate_ * dt;
      setSpinRef();
      return ev;
    }
    case DemoPhase::Settle: {
      rate_ = acc_ = 0.0f;
      setSpinRef();
      if (t_ >= params::kDemoSettleS) {
        phase_ = DemoPhase::Off;
        return Event::Complete;
      }
      return Event::None;
    }
    default:
      return Event::None;
  }
}

DemoSpin::Event DemoSpin::requestStop() {
  switch (phase_) {
    case DemoPhase::Level:
    case DemoPhase::LevelHold:
      phase_ = DemoPhase::Off;
      return Event::Complete;
    case DemoPhase::SpinUp:
    case DemoPhase::Hold:
      downFrom_ = rate_;
      phase_ = DemoPhase::SpinDown;
      t_ = 0.0f;
      return Event::SpinDownStarted;
    default:
      return Event::None;
  }
}

float DemoSpin::wheelBias() const {
  if (levelOnly_) return 0.0f;  // DEMO LEVEL: full wheel speed range for holding level
  switch (phase_) {
    case DemoPhase::Off:
      return 0.0f;
    case DemoPhase::Settle: {
      const float ramp = 0.8f * params::kDemoSettleS;  // reach 0 before the phase ends
      const float f = 1.0f - t_ / ramp;
      return f > 0.0f ? params::kDemoWheelBiasRadS * f : 0.0f;
    }
    default:
      return params::kDemoWheelBiasRadS;
  }
}

char DemoSpin::phaseChar() const {
  switch (phase_) {
    case DemoPhase::Level: return 'L';
    case DemoPhase::SpinUp: return 'U';
    case DemoPhase::Hold: return 'H';
    case DemoPhase::SpinDown: return 'D';
    case DemoPhase::Settle: return 'S';
    case DemoPhase::LevelHold: return 'K';
    default: return '-';
  }
}
