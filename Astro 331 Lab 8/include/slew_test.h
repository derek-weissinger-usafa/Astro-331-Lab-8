#pragma once
#include <stdint.h>

#include "quat_math.h"

// SLEW TEST (stretch requirement): slew through kSlewNumTargets attitudes along the shortest
// rotation (eigenaxis) from wherever the table starts, reaching each within +/-kSlewTolDeg in
// roll, pitch and yaw and holding it for kSlewHoldS.
//
// Targets are roll / pitch (absolute, from level) and yaw RELATIVE to the heading at start.
// Defaults in control_params.h; SLEW SET changes them (tilt limited to kSlewMaxTiltDeg, below the
// supervisor's kMaxTiltDeg so a small overshoot doesn't disarm).
//
// Per target:
//   Slew    q_ref(s) = exp(s * phi) (x) q_from, s: 0 -> 1 on a cosine profile (zero rate at both
//           ends), phi = shortest rotation from q_from to the target (body frame). Duration is the
//           longest of: kSlewMinS, the time at kSlewMaxRateRadS peak rate, the time at
//           kSlewMaxAccelRadS2 peak acceleration.
//   Settle  reference = target; wait until |roll, pitch, yaw error| < kSlewTolDeg continuously for
//           kSlewSettleS (gives up after kSlewSettleTimeoutS and holds anyway, marked not settled).
//   Hold    kSlewHoldS at the target; the worst error per axis is recorded. PASS = settled and
//           every axis stayed within kSlewTolDeg for the whole hold.
// The next slew starts from this target (continuous reference). After the last hold the results
// are final (Complete) and the test keeps holding the last target (phase Done) until STOP / KILL.
// While it runs, main.cpp's supervisor uses kSlewMaxTiltDeg (25 deg) instead of kMaxTiltDeg: the
// bubble has no top, so the table is kept under 25 deg.
//
// YAW-ONLY mode (SLEW YAW, 2026-10-31): the stretch test as this pendulous table can do it. Every
// degree away from the rest tilt costs ~11 mN m of wheel torque, so tilted targets fill the wheels in
// under a second, and even SLEW TEST with yaw-only targets at a fixed roll/pitch stopped after 13.5 s
// (the rest tilt moves ~0.3 deg with heading). Here only yaw follows the slew profile (rotation about
// the vertical: the shortest path between attitudes that differ only in heading); the roll/pitch
// reference is the measured tilt every step, so those axes are only damped (as DEMO LEVEL) and the
// table stays at its rest tilt. Targets: the yaw of each SLEW target (roll/pitch ignored). Roll/pitch
// errors are measured from the rest tilt (a kDemoRestAvgS low-pass of the measured tilt), so they
// show how still the table holds; yaw error is from the target heading.
//
// TILT mode (SLEW TILT, 2026-11-01): tilted targets on a table balanced close to neutral (centre of
// mass ~1 mm below the bearing centre). Needs the gravity stiffness k (SET GRAV_K, N m/rad, from the
// swing period: k = I (2 pi / T)^2). Phases:
//   Scan    one slow 360 deg yaw turn with roll/pitch only damped (as SLEW YAW), fitting the rest tilt
//           as a function of heading: rest(psi) = a0 + a1 cos psi + a2 sin psi (psi from the start
//           heading). The one-per-turn part is a torque fixed in the room (air bearing, drafts).
//   Unload  before each target: gravity is the only external torque about x/y, so holding the table
//           tilted by delta from its rest drains the wheels' x/y momentum at k*delta. delta =
//           h_xy / (k kUnloadTauS), capped at kUnloadMaxDeg and rate-limited: the stored momentum
//           decays with time constant kUnloadTauS. Ends when |h_xy| < kUnloadDoneFrac of the
//           capacity (kUnloadDoneS in a row) or after kUnloadTimeoutS. Not graded.
//   Slew / Settle / Hold as SLEW TEST, to rest(psi_i) + (droll_i, dpitch_i) at heading psi_i: the
//           tilt targets are OFFSETS from the rest tilt (SLEW TSET), so the gravity torque each one
//           costs is known and small. Graded against those targets, +/-kSlewTolDeg in all axes.
// Gravity feedforward (gravityFF): k * (reference tilt - rest(reference heading)) about body x/y,
// added to the controller output, so holding a tilt doesn't need a steady error (Kp alone would sit
// k*delta/Kp off). It costs the wheels the same momentum; unloading is what gives it back.
// Budget check (tiltBudget): before starting, each target's momentum k*|delta|*dwell is compared with
// the x/y wheel capacity; main.cpp refuses targets above kTiltBudgetFrac of it.
//
// Conventions as frames.h: q is inertial->body; reference rates are body rates in the REFERENCE
// body frame (the controller rotates them into the current body frame), as in demo_spin.h.
// Errors are 3-2-1 Euler differences (deg): estimate minus target, yaw wrapped to +/-180.
class SlewTest {
 public:
  static constexpr int kMaxTargets = 4;
  enum class Mode : uint8_t { Absolute, YawOnly, Tilt };
  enum class Phase : uint8_t { Off, Scan, Unload, Slew, Settle, Hold, Done };
  enum class Event : uint8_t { None, ScanDone, UnloadDone, SlewDone, Settled, SettleTimeout, TargetDone, Complete };

  struct Target {
    float rollDeg = 0, pitchDeg = 0, yawDeg = 0;  // yaw relative to the heading at start
  };
  struct Result {
    bool settled = false;
    float settleS = 0;        // time from end of slew to settled (s)
    Vec3 maxErrDeg{0, 0, 0};  // worst |roll|, |pitch|, |yaw| error during the hold
    bool pass = false;
    float restRollDeg = 0, restPitchDeg = 0;  // yaw-only: rest tilt at the end of the hold; tilt: rest(psi_i)
  };

  SlewTest();

  // Replace target i (0-based). Returns false (unchanged) if i is out of range or the tilt
  // exceeds kSlewMaxTiltDeg.
  bool setTarget(int i, float rollDeg, float pitchDeg, float yawDeg);
  const Target& target(int i) const { return targets_[i]; }
  // Tilt-mode targets: roll/pitch OFFSETS from the rest tilt (|offset| <= kTiltMaxOffsetDeg).
  bool setTiltTarget(int i, float dRollDeg, float dPitchDeg, float yawDeg);
  const Target& tiltTarget(int i) const { return tiltTargets_[i]; }
  // Momentum target i would store (N m s) with stiffness k, and the capacity it is compared with.
  float tiltBudget(int i, float k) const;
  static float tiltCapacity();

  void start(const Quat& qNow, Mode mode = Mode::Absolute, float gravK = 0.0f);
  // hWheels: wheel angular momentum in the body frame (N m s); used by the tilt mode's unloading.
  Event step(float dt, const Quat& qEst, const Vec3& hWheels = {0, 0, 0});
  void abort() { phase_ = Phase::Off; }

  bool active() const { return phase_ != Phase::Off; }
  Phase phase() const { return phase_; }
  char phaseChar() const;  // - off, C scan, U unload, S slew, W settle, H hold, D done (holding last)
  bool done() const { return phase_ == Phase::Done; }
  Mode mode() const { return mode_; }
  bool yawOnly() const { return mode_ == Mode::YawOnly; }
  bool tilt() const { return mode_ == Mode::Tilt; }
  int index() const { return idx_; }  // current target, 0-based
  int passCount() const;
  const Result& result(int i) const { return results_[i]; }
  float slewDuration() const { return T_; }
  float slewAngleDeg() const { return norm(phi_) * kRadToDeg; }
  float elapsed() const { return elapsed_; }
  Vec3 errorDeg() const { return errDeg_; }  // latest estimate - target (roll, pitch, yaw), deg
  float refRate() const { return norm(refW_); }
  float refYawDeg() const { return qToEuler321(refQ_).z * kRadToDeg; }
  const Quat& refQ() const { return refQ_; }
  const Vec3& refW() const { return refW_; }
  const Vec3& refA() const { return refA_; }
  // Tilt mode: gravity feedforward torque (body, N m) for the controller; zero in other modes.
  Vec3 gravityFF() const { return ff_; }
  // Tilt mode: fitted rest tilt (deg) at heading psiDeg (relative to the start heading), and the
  // fit's coefficients a0, a1 (cos), a2 (sin) for roll and pitch (deg).
  void restAt(float psiDeg, float& rollDeg, float& pitchDeg) const;
  const float* restRollFitDeg() const { return fitRollDeg_; }
  const float* restPitchFitDeg() const { return fitPitchDeg_; }
  float unloadOffsetDeg() const { return norm(delta_) * kRadToDeg; }
  float lastUnloadS() const { return lastUnloadS_; }
  float hxyAtUnloadStart() const { return hUnload0_; }

 private:
  void beginSlew(const Quat& qFrom);
  void beginUnload();
  Quat targetQuat(int i) const;
  void updateError(const Quat& qEst);
  Event advance(float dt, const Quat& qEst, const Vec3& hWheels);  // phase logic (errDeg_ already updated)
  // Yaw-only reference: measured tilt (roll, pitch) at heading yaw, turning at yawRate / yawAcc
  // about the vertical (body rates of a 3-2-1 yaw rate with no roll/pitch rate).
  void setYawRef(float roll, float pitch, float yaw, float yawRate, float yawAcc);
  void restRad(float psiRel, float& roll, float& pitch) const;
  void solveRestFit();

  Target targets_[kMaxTargets];
  Target tiltTargets_[kMaxTargets];
  Result results_[kMaxTargets];
  Phase phase_ = Phase::Off;
  Mode mode_ = Mode::Absolute;
  float gravK_ = 0;                       // tilt: gravity stiffness, N m/rad
  float yawFrom_ = 0, dYaw_ = 0;          // yaw-only: slew start heading and turn (rad)
  float rollRest_ = 0, pitchRest_ = 0;    // yaw-only: low-passed measured tilt (rad)
  int idx_ = 0;
  float yaw0_ = 0;     // heading at start (rad); target yaws are relative to it
  float t_ = 0;        // time in the current phase (s)
  float elapsed_ = 0;  // since start (s)
  float okT_ = 0;      // time continuously within tolerance (settle; unload: momentum low)
  Quat qFrom_{1, 0, 0, 0}, qTarget_{1, 0, 0, 0};
  float tgtRoll_ = 0, tgtPitch_ = 0;      // current target's absolute roll/pitch (rad)
  Vec3 phi_{0, 0, 0};  // slew rotation vector (body frame)
  float T_ = 0;        // slew / scan duration (s)
  Vec3 errDeg_{0, 0, 0};
  Quat refQ_{1, 0, 0, 0};
  Vec3 refW_{0, 0, 0}, refA_{0, 0, 0};
  Vec3 ff_{0, 0, 0};
  // Rest-tilt scan: normal equations for [1, cos psi, sin psi] per axis.
  float nrm_[3][3] = {{0}};
  float rhsRoll_[3] = {0}, rhsPitch_[3] = {0};
  float fitRollDeg_[3] = {0}, fitPitchDeg_[3] = {0};
  float fitRoll_[3] = {0}, fitPitch_[3] = {0};  // rad
  // Unloading
  float lpRoll_ = 0, lpPitch_ = 0;  // scan: low-passed measured tilt (rad), the roll/pitch reference
  float unloadYaw_ = 0;  // heading held while unloading (rad, absolute)
  Vec3 delta_{0, 0, 0};  // current unload tilt offset (x = roll, y = pitch, rad)
  float lastUnloadS_ = 0, hUnload0_ = 0;
};
