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

// ---- Wheels (motors run from the ~5 V Teensy rail for now) ----
constexpr float kWheelMaxSpeedRadS = 35.0f;   // command limit; ~80% of free speed
constexpr float kWheelFreeSpeedRadS = 44.0f;  // TODO: measure at the actual supply voltage
// Manual (SPEED) mode feedforward only. Measured 2026-10-08 on the USB/VIN 5 V supply:
// TEST 0.3 gave 24-28 rad/s. Re-measure with TEST after moving to the 12 V packs.
constexpr float kManualFreeSpeedRadS = 88.0f;
constexpr float kWheelOverspeedRadS = 40.0f;  // measured speed above this => disarm
constexpr float kWheelCmdLeadRadS = 8.0f;     // setpoint may lead measured speed by at most this
constexpr float kNullBiasRadS = 0.0f;        // 4-wheel null-mode speed target (0 = just hold it)
constexpr float kNullGain = 5.0f;             // 1/s

// Per-wheel signs, found during bring-up (see CLAUDE.md). +1 = as wired.
//  Positive wheel speed is a right-hand rotation about the wheel axis in wheel_geometry.h.
constexpr float kMotorSign[4] = {1.0f, 1.0f, 1.0f, 1.0f};    // + duty -> + speed?
// Bring-up 2026-10-08: +duty spins all four CCW (= +axis), so kMotorSign stays +1. Encoders
// counted negative (A, C, D measured; B inferred from the hand-spin test). D reads +26 rad/s
// at TEST 0.3 after the flip; re-check A-C read positive before trusting SPEED.
constexpr float kEncoderSign[4] = {-1.0f, -1.0f, -1.0f, -1.0f};  // + count -> + speed?

// ---- Wheel speed loop (inner, 100 Hz) ----
constexpr float kSpeedLoopHz = 100.0f;
constexpr float kSpeedKp = 0.02f;        // duty per rad/s
constexpr float kSpeedKi = 0.10f;        // duty per (rad/s) per s
constexpr float kSpeedIntegralClamp = 0.3f;  // duty
constexpr float kMaxDuty = 0.8f;
constexpr float kDutySlewPerSec = 5.0f;  // limits current steps on the shared 5 V rail
constexpr float kSpeedFilterAlpha = 0.4f;
constexpr float kEncoderCountsPerRev = 640.0f;  // 64 CPR at the motor * 10:1 gearbox
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

}  // namespace params
