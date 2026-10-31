#pragma once
#include <stdint.h>

#include "frames.h"

// Controller / actuator parameters. Almost all of these are PLACEHOLDERS to be
// replaced by measured values (inertias) or tuned on hardware (gains, limits).
namespace params {

// ---- Plant ----
// Table: working MODEL until measured. Hollow PLA cube (15 in side, 1 in wall) scaled uniformly
// to 5 kg => ~8.9 in side, 0.59 in wall, I = m a^2/6 shell => 0.0623 kg m^2 about each axis.
constexpr float kBodyInertia[3] = {0.0623f, 0.0623f, 0.0623f};  // kg m^2, about body x, y, z
// Wheel: from CAD "CAD files V2/ReactionWheel V2.f3d", aluminium 2700 kg/m^3: 161 g, rim
// r 4-5 cm x 1.9 cm (97% of J), 4 mm hub plate, four 8 mm spokes.
// TODO (rotor momentum): the motor rotor spins 6.25x faster than the wheel and adds roughly
// N * J_rotor to each wheel's momentum (a few percent). Not modelled yet; it only adds margin.
constexpr float kWheelInertia = 3.06e-4f;                 // kg m^2, axial, per wheel

// ---- Attitude loop ----
// Kp = I wn^2, Kd = 2 zeta I wn per axis. Hold target: level, yaw 0 (+/-5 deg spec).
constexpr float kAttBandwidthRadS = 2.0f;
constexpr float kAttDamping = 0.9f;
constexpr float kAttKi = 0.0f;       // N m/(rad s). Wheels already integrate constant torques.
constexpr float kAttIntegralClamp = 0.2f;  // rad s

// ---- Wheels (each motor on its own 3S Epoch 18650 pack: 12.6 V full, 11.1 V nominal, ~9 V empty) ----
// Gearmotors are Pololu 37D **6.3:1** (gearbox label GB37-6.3), not 10:1: confirmed 2026-10-15
// by hand-turning each wheel 10 turns with COUNTS (~400 counts/turn). ~5.5 A stall at 12 V
// (TODO: verify). Wheel speed per unit duty, measured 2026-10-15 on the 12 V packs: B and D held
// a true ~50 rad/s at 0.30-0.31 duty => ~160-167 rad/s per unit duty (~1600 rpm free at 12 V).
// (Earlier figures of 88 and 100 were in mis-scaled 10:1 units and are superseded.)
constexpr float kWheelFreeSpeedRadS = 160.0f;  // speed-loop feedforward + back-EMF current limit
// Command limit (~764 rpm; was 60 until 2026-10-31). The accelerating torque left at speed w is
// min(I_limit/I_stall = 0.45, kMaxDuty - w/free speed): full torque up to ~79 rad/s on a 12 V pack
// (~90% on a sagging ~11 V pack). Above ~90 it falls off fast (71% at 100, 44% at 120), and wheel D
// (~25% slower than A-C at the same duty, 2026-10-31) has less. Raise toward ~100 only after D's
// drag is fixed and a clamped run at the new speed shows no vibration problem.
constexpr float kWheelMaxSpeedRadS = 80.0f;
// Manual (SPEED) mode feedforward. Same motors, so same measured value as above.
constexpr float kManualFreeSpeedRadS = 160.0f;
constexpr float kWheelOverspeedRadS = 90.0f;   // measured speed above this => disarm (kWheelMaxSpeedRadS + 10)
// Encoder fault detection: |duty| above kEncFaultDuty with |speed| below kEncFaultSpeedRadS for
// kEncFaultTimeS (the motor alone reaches ~160 rad/s per unit duty), or a speed change faster
// than kEncFaultMaxAccelRadS2 (stall torque / wheel J is ~1000 rad/s^2), disables the wheel.
// 0.4, not 0.2 (2026-10-31): a healthy wheel stuck at rest by static friction can sit at 0.2-0.3
// duty for over 0.5 s (wheel D, after the dead-motor detector got the same fix). A lost encoder
// reads 0, so the speed loop still winds up to the current limit (0.45 at rest) and trips this.
constexpr float kEncFaultDuty = 0.4f;
constexpr float kEncFaultSpeedRadS = 1.0f;
constexpr float kEncFaultTimeS = 0.5f;
constexpr float kEncFaultMaxAccelRadS2 = 3000.0f;
constexpr float kWheelCmdLeadRadS = 13.0f;    // setpoint may lead measured speed by at most this
constexpr float kNullBiasRadS = 0.0f;        // 4-wheel null-mode speed target (0 = just hold it)
constexpr float kNullGain = 5.0f;             // 1/s
// Heading holds (ARM, DEMO LEVEL, all four wheels): the null mode keeps every wheel at least this
// far from zero (null_bias.h) so the magnetometer aid can stay in use. BIAS <rpm> changes it, 0 = off.
constexpr float kHoldWheelMinRadS = 10.47f;  // 100 rpm (also clear of the ~50 rpm resonance)
// ...but the bias never drives a wheel above this (under kDemoWheelAbortRadS), so it can't end a
// hold early: with roll/pitch momentum stored (pendulous table) full clearance would cost peak speed.
constexpr float kHoldBiasMaxPeakRadS = 65.0f;  // kDemoWheelAbortRadS - 10

// Per-wheel signs, found during bring-up (see CLAUDE.md). +1 = as wired.
//  Positive wheel speed is a right-hand rotation about the wheel axis in wheel_geometry.h.
constexpr float kMotorSign[4] = {1.0f, 1.0f, 1.0f, 1.0f};    // + duty -> + speed?
// Bring-up 2026-10-08: +duty spins all four CCW (= +axis), so kMotorSign stays +1. Encoders
// counted negative (A, C, D measured; B inferred from the hand-spin test). D read +26 rad/s
// at TEST 0.3 after the flip (old 640-count scaling; ~42 rad/s true); re-check A-C read positive.
constexpr float kEncoderSign[4] = {-1.0f, -1.0f, -1.0f, -1.0f};  // + count -> + speed?

// ---- Wheel speed loop (inner, 100 Hz) ----
// Gains scaled by 100/160 when the encoder scaling was corrected (2026-10-15): speeds now read
// 1.6x larger, so this keeps the same loop behavior as the 2026-10-15 bench tests.
constexpr float kSpeedLoopHz = 100.0f;
constexpr float kSpeedKp = 0.0056f;      // duty per rad/s
constexpr float kSpeedKi = 0.028f;       // duty per (rad/s) per s
constexpr float kSpeedIntegralClamp = 0.3f;  // duty
constexpr float kMaxDuty = 0.95f;        // headroom to reach kWheelMaxSpeedRadS on a sagging pack
constexpr float kDutySlewPerSec = 5.0f;  // smooths current steps drawn from the packs

// Motor current limit without current sensing. Motor current is roughly
//   i ~= I_stall * (duty - speed / free_speed)
// so keeping |duty - speed/free_speed| <= I_limit / I_stall bounds both drive and
// braking current. Applied in WheelMotor::setDuty (closed loop and TEST alike).
constexpr float kMotorStallCurrentA = 5.5f;  // at 12 V, TODO: verify
constexpr float kMotorCurrentLimitA = 2.5f;  // below the TB9051FTG continuous rating
constexpr float kSpeedFilterAlpha = 0.4f;
constexpr float kEncoderCountsPerRev = 400.0f;  // 64 CPR at the motor * 6.25 (6.3:1 gearbox), measured
constexpr float kPwmFreqHz = 20000.0f;          // TODO: confirm against the TB9051FTG limit
constexpr int kPwmBits = 12;

// ---- Safety limits (supervisor disarms the wheels when exceeded) ----
constexpr float kMaxTiltDeg = 30.0f;     // roll or pitch
constexpr float kMaxRateRadS = 2.0f;
// Gyro gaps up to this are integrated (main.cpp); longer ones are dropped, and disarm when Armed.
// 300 ms, not 100 (2026-10-31): with SD logging on, loop() stalls in pairs every ~1.67 MB of log
// (~53 s) and the BNO085 drops 5-12 reports (gaps 60-130 ms, up to 140 ms at log open; STATIC_0008
// 31 Oct, DEMO_0004/0005), independent of wheel speed. Gaps over 100 ms were skipped, losing the
// rotation in them (~0.75 deg at a 0.1 rad/s slew).
constexpr uint32_t kImuTimeoutMs = 300;

// ---- Rates ----
// Sensor collection and control run at 100 Hz: the gyro report (imu_bno085.cpp) drives
// EKF predict + the attitude step, and the wheel speed loop runs at kSpeedLoopHz.

// ---- Gyro bias calibration (at boot and on CALIBRATE) ----
// The z (yaw) bias can only come from this static average (gravity carries no heading info).
// Static test 2026-10-15: z noise 2.17e-4 rad/s/sqrt(Hz) => bias error ~ noise/sqrt(T):
// 3 s -> ~26 deg/h of yaw drift (measured 27 deg/h); 60 s -> ~6 deg/h.
constexpr uint32_t kCalDurationMs = 60000;
constexpr uint32_t kCalProgressMs = 10000;   // progress message interval

// ---- Static test ("BEGIN STATIC TEST [rpm]") ----
// Wheel speed held during the static test, to measure magnetometer / IMU behaviour with the
// motors running (all four wheels, Manual mode). 0 = motors off.
constexpr float kStaticTestWheelRpm = 100.0f;

// ---- Magnetometer disturbance map ("MAG MAP", schedule in mag_map.h) ----
constexpr uint32_t kMagMapStepMs = 20000;  // hold per step (ramp ~1-2 s + settled average)

// ---- IMU mounting (LEVEL ZERO) ----
// Roll/pitch the IMU reads when the DECK is level (spirit level, table clamped): every IMU sample
// is rotated by this into the table frame, so the reported attitude (requirement 2) is the
// table's, not the IMU board's. Measured with LEVEL ZERO 2026-10-31 after the IMU was glued to its mount
// (it could rock before, which made the balance rest-tilt readings jump); deck levelled with a phone
// level calibrated by reversal. Re-measured ~1.5 h later (pitch had moved 1.1 deg: the glue was still
// creeping; first value 3.649 / -1.033). Re-check with the phone level once it has cured, and if the IMU
// is ever remounted.
constexpr float kImuMountRollDeg = 3.611f;
constexpr float kImuMountPitchDeg = -2.164f;
constexpr float kLevelZeroS = 5.0f;  // LEVEL ZERO averaging time

// ---- Heading step responses on the air bearing ("STEP TEST", step_test.h) ----
constexpr float kStepDefaultDeg = 5.0f;
constexpr float kStepMaxDeg = 30.0f;
constexpr float kStepHoldS = 20.0f;      // each segment (baseline and every step)
constexpr float kStepTailS = 5.0f;       // steady-state error / jitter window at the end of each
constexpr float kStepBandMinDeg = 0.2f;  // settling band: max(10% of the step, this)
constexpr float kStepIdS = 6.0f;         // inertia fit window after each step (covers the move)

// ---- Yaw-aid validation on the fixed table ("MAG TEST", schedule in mag_test.h) ----
constexpr float kMagTestTolDeg = 1.0f;       // |yaw| limit (the table is fixed, so true yaw = 0)
constexpr float kMagTestRecoverS = 90.0f;    // KICK steps: back within the limit this soon (tau ~25 s)
constexpr float kMagTestDriftMin = 30.0f;    // default drift-run length (MAG TEST [min] overrides)
constexpr float kMagTestDriftMaxMin = 240.0f;

// ---- Magnetometer heading aid (mag_aid.h; calibrated 2026-10-27) ----
constexpr float kMagLpfTauS = 0.5f;         // low-pass on the corrected field (motor noise is 3-50 Hz)
constexpr float kMagStillRadS = 0.05f;      // all enabled wheels below this (and counts fixed) = Stopped
// Spinning = every enabled wheel's low-passed |speed| above kMagSpinEnterRadS; it stays Spinning
// until one drops below kMagSpinExitRadS (hysteresis: wheel C's speed oscillates +-30-90 rpm).
constexpr float kMagSpinEnterRadS = 6.0f;   // ~57 rpm
constexpr float kMagSpinExitRadS = 4.0f;    // ~38 rpm
constexpr float kMagSpeedLpfTauS = 0.3f;
// A reference may only be learned if a heading update was applied within this long (or the
// startup calibration just ended), so it doesn't inherit drifted yaw. Gyro drift <= ~25 deg/h
// => <= 0.2 deg in 30 s.
constexpr float kMagLearnMaxGapS = 30.0f;
// New spinning direction patterns: learn the reference in place (false) or always predict it from
// a known one (true). Prediction is still used when the aid is stale. Two fixed-table runs disagree
// (mag_aid.h): MAGTEST_0001 learned 36/37 (1.02 deg) vs predicted 37/37 (0.77); MAGTEST_0002
// learned 15/15 (0.78) vs predicted, as run, 10/15 (1.49): the reversal model moved ~1 deg between
// sessions. Holds switch pattern only once or twice (null_bias.h), which favours learning.
constexpr bool kMagPredictNewPatterns = false;
constexpr float kMagRefSpeedRadS = 10.47f;  // speed model reference (100 rpm)
constexpr float kMagSettleStillS = 1.5f;    // after the wheels stop / start before the field is used
constexpr float kMagSettleSpinS = 2.0f;
constexpr float kMagLearnStillS = 3.0f;     // reference averaging time per state
constexpr float kMagLearnSpinS = 10.0f;
constexpr long kMagStoppedCountTol = 1;     // encoder counts a stopped wheel may differ from its reference
constexpr float kMagHorizTolFrac = 0.15f;   // gate if the horizontal field strength is off by more
// Heading noise fed to the EKF per sample (100 Hz). Deliberately large: the low-passed samples are
// strongly correlated, and the aid should only remove slow gyro drift (~30 s time constant).
constexpr float kMagHeadingSigmaRad = 0.3f;
// Table hard iron (body frame, uT), removed before the field is used. Not fitted yet: needs a yaw
// turn on the air bearing (tools/hardiron_fit.py); SET MAG_HX/HY/HZ tries a value without an upload.
constexpr float kMagHardIronUt[3] = {0.0f, 0.0f, 0.0f};

// ---- IMU sample-loss diagnostics ----
constexpr float kImuGapMs = 15.0f;  // gyro interval above this (nominal 10 ms) counts as a gap

// ---- Data log (SD card + XBee) / telemetry / bench ----
constexpr uint32_t kXbeeBaud = 9600;       // ~960 B/s budget
// One log row per gyro sample (~100 Hz). Rows are ~230-260 B, so the 9600-baud XBee gets
// every 50th row (~2 Hz, ~500 B/s) to leave room for command replies.
constexpr uint32_t kXbeeLogDecimation = 50;
constexpr uint32_t kSdFlushMs = 1000;      // max data lost on power cut
constexpr uint32_t kUsbTelemetryMs = 50;
constexpr float kTestMaxDuty = 0.5f;
constexpr uint32_t kTestDurationMs = 1000;
constexpr float kManualAccelRadS2 = 20.0f;  // SPEED setpoint ramp rate (~190 rpm/s)
// Extra first-order low-pass on measured wheel speed, Manual mode only (speed loop + USB
// telemetry). Larger = smoother but more lag (more overshoot risk). 0 disables it.
constexpr float kManualSpeedLpfTauS = 0.1f;
// A Manual wheel with target 0, ramp finished and |speed| below this is released (no drive, PI reset).
constexpr float kManualReleaseRadS = 0.3f;

// ---- Demo: level, then spin counter-clockwise about +z (XBee "DEMO SPIN", "STOP") ----
// Requested 30 rpm, but the rate actually used is capped by the wheels' z-momentum budget:
//   rate = margin * sum(a_i.z) * J * (kWheelMaxSpeedRadS - kDemoWheelBiasRadS) / I_zz
//        (~0.77 rad/s = 7.4 rpm with the 100 rpm wheel bias; 8.5 rpm without it, at 80 rad/s)
constexpr float kDemoRequestedRateRadS = 3.14159265f;  // 30 rpm
constexpr float kDemoMomentumMargin = 0.8f;    // leave 20% for levelling, drag, tracking error
constexpr float kDemoLevelRateRadS = 0.1f;     // max reference tilt rate while levelling (~6 deg/s)
constexpr float kDemoLevelMinS = 2.0f;         // shortest levelling slew
constexpr float kDemoLevelTolDeg = 1.0f;       // "level" = |roll|,|pitch| below this ...
constexpr float kDemoLevelHoldS = 3.0f;        // ... continuously for this long
// DEMO LEVEL holds the table's natural rest tilt (where gravity balances it), not exact level
// (DEMO_0003_OLD: holding exact level cost 1.3-3.3 mN m and filled a wheel in ~15 s). Roll/pitch are
// only damped; the rest tilt is reported as a low-pass of the measured tilt with this time constant.
constexpr float kDemoRestAvgS = 2.0f;
constexpr float kDemoRestTolDeg = 0.25f;       // "settled": |tilt - rest average| below this for kDemoLevelHoldS
constexpr float kDemoLevelTimeoutS = 20.0f;    // after the slew; then abort
constexpr float kDemoRampS = 10.0f;            // spin-up and spin-down duration
constexpr float kDemoHoldS = 10.0f;            // constant-rate hold
constexpr float kDemoSettleS = 5.0f;           // hold heading at rest (and ramp the wheel bias to 0)
// Wheel bias during the demo (MAG MAP 2026-10-16): stopped motors leave a static magnetometer
// offset that depends on where each shaft stops (up to ~5 uT / ~5 deg per wheel), and slow
// wheels swing the field too slowly to filter. So the demo holds the 4-wheel null mode
// [+1,-1,+1,-1] at this speed: A,C at +100 rpm, B,D at -100 rpm (no net torque or momentum),
// keeping the magnet fields fast and averaging out. 100 rpm also avoids the ~50 rpm vibration
// resonance. Costs speed headroom: the spin rate budget uses (kWheelMaxSpeedRadS - bias).
// The z spin drives all wheels the same way, so A and C cross zero briefly on each ramp.
constexpr float kDemoWheelBiasRadS = 10.47f;   // 100 rpm
constexpr float kDemoStartMaxWheelRadS = 2.0f; // wheels must be ~stopped to start
constexpr float kDemoWheelAbortRadS = 75.0f;   // any wheel above this => spin down early (max - 5)

// ---- SLEW TEST (stretch requirement; slew_test.h) ----
// Four attitudes: roll, pitch (deg from level) and yaw (deg, relative to the heading at start).
// Tilt must stay <= kSlewMaxTiltDeg: the bubble has no top, so the team keeps the table under
// 25 deg; during SLEW TEST the supervisor also disarms above 25 deg (instead of kMaxTiltDeg). Change at run time with SLEW SET <1-4> <roll> <pitch> <yaw>.
// NOTE: on a pendulous table every degree of tilt away from the rest tilt costs a steady
// gravity torque (~5 mN m/deg after the 2026-10-21 balancing) that the wheels can't hold for
// long; see PROGRESS_LOG.md. Tilted targets need the CG very close to the bearing centre.
constexpr int kSlewNumTargets = 4;
constexpr float kSlewTargets[kSlewNumTargets][3] = {
    {10.0f, 0.0f, 45.0f},    // roll right, quarter turn left of start
    {0.0f, -10.0f, 135.0f},  // nose down
    {-8.0f, 8.0f, -135.0f},  // roll left + nose up (shortest path crosses 180)
    {0.0f, 0.0f, 0.0f},      // back to level at the start heading
};
constexpr float kSlewMaxTiltDeg = 25.0f;
constexpr float kSlewMaxRateRadS = 0.1f;     // peak reference rate along the slew (~5.7 deg/s)
constexpr float kSlewMaxAccelRadS2 = 0.02f;  // peak reference acceleration
constexpr float kSlewMinS = 3.0f;            // shortest slew
constexpr float kSlewTolDeg = 1.0f;          // requirement: +/-1 deg in roll, pitch and yaw
constexpr float kSlewSettleS = 1.0f;         // within tolerance this long = "reached"
constexpr float kSlewSettleTimeoutS = 20.0f; // give up waiting and hold anyway (marked not settled)
constexpr float kSlewHoldS = 5.0f;           // requirement: hold each attitude 5 s

// ---- Wheel failure: dead-motor detection + FAULT TEST (requirement 3; wheel_fault.h) ----
// Drive effort e = duty - speed/kWheelFreeSpeedRadS (~ motor current / stall current). A working
// motor accelerates its wheel at ~kMotorAccelPerEffort * e less friction: stall torque (~0.24-0.29
// N m for the 6.3:1 37D at 12 V, TODO: verify) over the wheel J, de-rated ~15% for the motor rotor
// (rotor momentum TODO). Each kDeadBlockS block: mean |e| >= kDeadEffort and less than
// kDeadResponseFrac of the predicted speed gain in the effort direction = unresponsive;
// kDeadDetectS of unresponsive blocks in a row = dead motor (disabled, then coasts).
// Margins (sim_fault.py): a healthy wheel at e = 0.2 gains >= ~120 rad/s^2; the 0.25 x 800 x 0.2
// = 40 rad/s^2 threshold sits above friction-only deceleration (~20-30 rad/s^2, rubbing wheel).
// Simulated detection: 0.6-0.8 s after the failure; no false trips in healthy runs.
// Standstill (block starts and ends below kDeadStillRadS): a healthy wheel only breaks away from
// rest at effort ~0.10-0.15, sometimes up to ~0.27 (static friction; DEMO_0004/0005, 230 starts),
// so there a block counts only at mean |e| >= kDeadStillEffort, just under the current-limit cap
// (kMotorCurrentLimitA / kMotorStallCurrentA = 0.45). With 0.2 everywhere, 14 healthy starts were
// flagged dead (13 on wheel D, which sticks more), all with the wheel stuck at e 0.23-0.30.
// Replaying the two logs with this rule: 0 flags. A dead motor at rest still trips once the speed
// loop winds up to the current limit; a spinning one (FAULT TEST's wheel bias) is unaffected.
constexpr float kMotorAccelPerEffort = 800.0f;  // rad/s^2 per unit effort
constexpr float kDeadEffort = 0.2f;
constexpr float kDeadStillRadS = 2.0f;
constexpr float kDeadStillEffort = 0.4f;
constexpr float kDeadResponseFrac = 0.25f;
constexpr float kDeadBlockS = 0.1f;
constexpr float kDeadDetectS = 0.2f;
// FAULT TEST <A-D> [delay]: after the delay (counted while Armed), as soon as wheel X turns at
// >= kFaultMinWheelRadS (an idle, unneeded wheel can't show a failure), its motor is made dead
// without telling the controller; whatever is running carries on. Graded: detected within
// kFaultDetectMaxS on the right wheel; attitude error back within +/-kFaultTolDeg (all axes,
// continuously kFaultSettleS) within kFaultRecoverMaxS of the failure; then held within
// +/-kFaultTolDeg for kFaultHoldS.
// While the test runs the 4-wheel null mode holds A,C at +100 rpm and B,D at -100 rpm (no net
// torque or momentum, as in DEMO SPIN): a stopped motor that is never asked for torque looks the
// same dead or alive, so the wheels are kept turning to make a dead one show up at once.
constexpr float kFaultDelayS = 10.0f;
constexpr float kFaultTolDeg = 1.0f;
constexpr float kFaultSettleS = 1.0f;
constexpr float kFaultDetectMaxS = 1.0f;
constexpr float kFaultRecoverMaxS = 10.0f;
constexpr float kFaultHoldS = 30.0f;
constexpr float kFaultWheelBiasRadS = 10.47f;  // 100 rpm
constexpr float kFaultMinWheelRadS = 3.0f;

}  // namespace params
