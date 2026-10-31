#pragma once
#include <stdint.h>

// "STEP TEST [deg]": heading step responses on the air bearing, to tune the attitude loop
// (requirement 1: hold a heading with no visible jitter) and to measure the table's yaw inertia.
// Runs on top of DEMO LEVEL (demo_spin.h): roll/pitch are only damped at the rest tilt, so the
// pendulous table doesn't fill the wheels, and the held heading is stepped:
//   baseline kStepHoldS at the start heading, then +A, 0, -A, 0 (each held kStepHoldS)
// Per step (response = heading change from the step's start, as a fraction of the step):
//   rise      10% -> 90% time
//   overshoot max beyond the target, % of the step
//   settle    time of the last sample outside the band max(10% of the step, kStepBandMinDeg)
//   error     mean heading error over the last kStepTailS of the hold
//   jitter    rms heading about its own mean over the last kStepTailS (also for the baseline)
//   Izz       yaw inertia from momentum exchange over the first kStepIdS: gravity gives no yaw
//             torque, so I_zz omega_z + h_wheels,z changes only by the yaw drag tau_d. Integrated:
//               I_zz * (psi - psi0) + integral(h_z - h_z0) dt = tau_d t^2 / 2
//             fitted by least squares for I_zz and tau_d (integrating makes the encoder speed
//             filter's lag negligible). Scales with the wheel inertia used (kWheelInertia, CAD; the
//             motor rotors add a few % that isn't modelled), so read it as "I_zz in wheel-J units".
// Gains can be changed between runs with SET ATT_BW / ATT_ZETA / IZZ (tune.h).
class StepTest {
 public:
  enum class Event : uint8_t { None, BaselineDone, StepDone, Complete };

  struct Result {
    float targetDeg = 0;   // step size (0 for the baseline)
    float riseS = -1;      // -1 = never reached 90%
    float overshootPct = 0;
    float settleS = -1;    // -1 = never settled
    float errDeg = 0;
    float jitterDeg = 0;
    float izz = 0;         // kg m^2, 0 if not measurable
    float dragMNm = 0;     // yaw drag torque from the same fit, mN m
  };

  void start(float ampDeg);
  void abort() { index_ = -1; }
  bool active() const { return index_ >= 0; }
  int index() const { return index_; }  // 0 = baseline, 1..4 = steps
  static constexpr int kSteps = 4;
  // Heading offset to hold now (rad), relative to the start heading.
  float offsetRad() const;

  // Per control step once the table has settled at its rest tilt. yawDeg: heading relative to the
  // start heading; omegaZ: body yaw rate (rad/s); hz: wheels' yaw momentum (N m s, body frame).
  Event update(float dt, float yawDeg, float omegaZ, float hz);

  const Result& lastResult() const { return last_; }
  float meanIzz() const { return nIzz_ ? sumIzz_ / nIzz_ : 0.0f; }

 private:
  void beginSegment(float yawDeg, float omegaZ, float hz);
  float targetDeg(int i) const;

  int index_ = -1;
  float amp_ = 5.0f;
  float t_ = 0;
  float y0_ = 0;          // heading at the segment start (deg)
  float w0_ = 0, h0_ = 0; // yaw rate / wheel momentum at the segment start
  float riseLo_ = -1, riseHi_ = -1, peak_ = 0, lastOut_ = 0;
  // tail statistics
  double tN_ = 0, tSum_ = 0, tSq_ = 0;
  // inertia fit: y = psi - psi0 (rad), q = t^2/2, x = integral(h - h0) dt;  I y - tau q = -x
  double sYY_ = 0, sYQ_ = 0, sQQ_ = 0, sYX_ = 0, sQX_ = 0, xInt_ = 0;
  Result last_;
  float sumIzz_ = 0;
  int nIzz_ = 0;
};
