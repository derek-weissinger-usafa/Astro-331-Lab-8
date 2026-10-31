#pragma once
#include <stdint.h>

// Runtime-tunable parameters: "SET <name> <value>" / "GET [name]" over the XBee or USB, so gains
// can be tuned on the air bearing without the USB-with-battery-off upload cycle (CLAUDE.md,
// "Uploading and running"). Each starts at its control_params.h value and is NOT saved: a reboot
// goes back to control_params.h, so copy values that work into that file. Ranges are clamped.
// Users read the variables below directly; version() changes on every SET (the attitude
// controller rebuilds its gains when it does).
namespace tune {

extern float attBw;         // attitude loop bandwidth, rad/s       (params::kAttBandwidthRadS)
extern float attZeta;       // attitude loop damping ratio           (params::kAttDamping)
extern float attKi;         // N m/(rad s)                           (params::kAttKi)
extern float inertia[3];    // table inertia about body x, y, z, kg m^2 (params::kBodyInertia)
extern float speedKp;       // wheel speed loop, duty per rad/s      (params::kSpeedKp)
extern float speedKi;       // duty per rad                          (params::kSpeedKi)
extern float magSigma;      // mag heading noise per sample, rad    (params::kMagHeadingSigmaRad)
extern float nullGain;      // null-mode speed loop, 1/s             (params::kNullGain)
extern float holdMinRadS;   // heading-hold wheel clearance, rad/s   (params::kHoldWheelMinRadS; BIAS)
extern float holdMaxPeak;   // ... never above this wheel speed      (params::kHoldBiasMaxPeakRadS)
extern float magHard[3];    // table hard iron, body x/y/z, uT       (params::kMagHardIronUt; mag_aid.h)
extern float imuRoll0Deg;   // IMU roll/pitch with the deck level    (params::kImuMount*Deg; LEVEL ZERO)
extern float imuPitch0Deg;

struct Param {
  const char* name;
  float* value;
  float lo, hi;
  const char* what;
};

int count();
const Param& at(int i);
const Param* find(const char* name);  // case-insensitive
// Clamps to the range; returns false for an unknown name.
bool set(const char* name, float value);
uint32_t version();

}  // namespace tune
