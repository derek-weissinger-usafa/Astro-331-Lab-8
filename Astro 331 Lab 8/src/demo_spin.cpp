#include "demo_spin.h"

#include "control_params.h"

static constexpr float kPi = 3.14159265f;

// Level attitude with yaw psi (inertial -> body): conj of an active rotation by psi about z.
static Quat levelQuat(float psi) { return qconj(qFromRotVec({0.0f, 0.0f, psi})); }

void DemoSpin::start(const Quat& qStart, float targetRate) {
  target_ = targetRate;
  rate_ = acc_ = 0.0f;
  t_ = 0.0f;
  levelOkT_ = 0.0f;
  yaw_ = qToEuler321(qStart).z;  // keep the current heading; only remove tilt

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

// Level, rotating about +z at rate_: w = (0, 0, rate) in the reference body frame.
void DemoSpin::setSpinRef() {
  ref_.q = levelQuat(yaw_);
  ref_.w = {0.0f, 0.0f, rate_};
  ref_.a = {0.0f, 0.0f, acc_};
}

DemoSpin::Event DemoSpin::step(float dt, float roll, float pitch) {
  const float T = params::kDemoRampS;
  t_ += dt;
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
      if (levelOkT_ >= params::kDemoLevelHoldS) {
        phase_ = DemoPhase::SpinUp;
        t_ = 0.0f;
        rate_ = acc_ = 0.0f;
        setSpinRef();
        return Event::Leveled;
      }
      if (t_ > levelT_ + params::kDemoLevelTimeoutS) return Event::LevelTimeout;
      return Event::None;
    }
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

char DemoSpin::phaseChar() const {
  switch (phase_) {
    case DemoPhase::Level: return 'L';
    case DemoPhase::SpinUp: return 'U';
    case DemoPhase::Hold: return 'H';
    case DemoPhase::SpinDown: return 'D';
    case DemoPhase::Settle: return 'S';
    default: return '-';
  }
}
