#pragma once
#include <stdint.h>

#include "frames.h"

// Controller / actuator parameters. Almost all of these are PLACEHOLDERS to be
// replaced by measured values (inertias) or tuned on hardware (gains, limits).
namespace params {

// ---- Plant (TODO: measure/estimate; the old Inertia.mat is for previous hardware) ----
constexpr float kBodyInertia[3] = {0.05f, 0.05f, 0.05f};  // kg m^2, about body x, y, z
constexpr float kWheelInertia = 2.0e-4f;                  // kg m^2, axial, per wheel

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

// ---- Data log (SD card + XBee) / telemetry / bench ----
constexpr uint32_t kXbeeBaud = 9600;       // ~960 B/s budget
constexpr float kLogHz = 20.0f;            // SD rows per second
constexpr uint32_t kXbeeLogDecimation = 4; // every 4th SD row to the XBee (5 Hz, ~500 B/s)
constexpr uint32_t kSdFlushMs = 1000;      // max data lost on power cut
constexpr uint32_t kUsbTelemetryMs = 50;
constexpr float kTestMaxDuty = 0.5f;
constexpr uint32_t kTestDurationMs = 1000;
constexpr float kManualAccelRadS2 = 20.0f;  // SPEED setpoint ramp rate (~190 rpm/s)
// Extra first-order low-pass on measured wheel speed, Manual mode only (speed loop + USB
// telemetry). Larger = smoother but more lag (more overshoot risk). 0 disables it.
constexpr float kManualSpeedLpfTauS = 0.1f;

}  // namespace params
