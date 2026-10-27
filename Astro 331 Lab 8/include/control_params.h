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
constexpr float kWheelMaxSpeedRadS = 60.0f;    // command limit (~573 rpm); well under free speed
// Manual (SPEED) mode feedforward. Same motors, so same measured value as above.
constexpr float kManualFreeSpeedRadS = 160.0f;
constexpr float kWheelOverspeedRadS = 70.0f;   // measured speed above this => disarm
// Encoder fault detection: |duty| above kEncFaultDuty with |speed| below kEncFaultSpeedRadS for
// kEncFaultTimeS (the motor alone reaches ~160 rad/s per unit duty), or a speed change faster
// than kEncFaultMaxAccelRadS2 (stall torque / wheel J is ~1000 rad/s^2), disables the wheel.
constexpr float kEncFaultDuty = 0.2f;
constexpr float kEncFaultSpeedRadS = 1.0f;
constexpr float kEncFaultTimeS = 0.5f;
constexpr float kEncFaultMaxAccelRadS2 = 3000.0f;
constexpr float kWheelCmdLeadRadS = 13.0f;    // setpoint may lead measured speed by at most this
constexpr float kNullBiasRadS = 0.0f;        // 4-wheel null-mode speed target (0 = just hold it)
constexpr float kNullGain = 5.0f;             // 1/s

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
constexpr uint32_t kImuTimeoutMs = 100;

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

// ---- Magnetometer heading aid (mag_aid.h; calibrated 2026-10-27) ----
constexpr float kMagLpfTauS = 0.5f;         // low-pass on the corrected field (motor noise is 3-50 Hz)
constexpr float kMagStillRadS = 0.05f;      // all enabled wheels below this (and counts fixed) = Stopped
constexpr float kMagMinWheelRadS = 5.0f;    // all enabled wheels above this (~50 rpm) = Spinning
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

// ---- Demo: level, then spin counter-clockwise about +z (XBee "DEMO SPIN", "STOP") ----
// Requested 30 rpm, but the rate actually used is capped by the wheels' z-momentum budget:
//   rate = margin * sum(a_i.z) * J * (kWheelMaxSpeedRadS - kDemoWheelBiasRadS) / I_zz
//        (~0.55 rad/s = 5.3 rpm with the 100 rpm wheel bias; 6.4 rpm without it)
constexpr float kDemoRequestedRateRadS = 3.14159265f;  // 30 rpm
constexpr float kDemoMomentumMargin = 0.8f;    // leave 20% for levelling, drag, tracking error
constexpr float kDemoLevelRateRadS = 0.1f;     // max reference tilt rate while levelling (~6 deg/s)
constexpr float kDemoLevelMinS = 2.0f;         // shortest levelling slew
constexpr float kDemoLevelTolDeg = 1.0f;       // "level" = |roll|,|pitch| below this ...
constexpr float kDemoLevelHoldS = 3.0f;        // ... continuously for this long
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
constexpr float kDemoWheelAbortRadS = 55.0f;   // any wheel above this => spin down early

}  // namespace params
