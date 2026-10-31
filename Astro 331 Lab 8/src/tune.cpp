#include "tune.h"

#include <ctype.h>

#include "control_params.h"

namespace tune {

float attBw = params::kAttBandwidthRadS;
float attZeta = params::kAttDamping;
float attKi = params::kAttKi;
float inertia[3] = {params::kBodyInertia[0], params::kBodyInertia[1], params::kBodyInertia[2]};
float speedKp = params::kSpeedKp;
float speedKi = params::kSpeedKi;
float magSigma = params::kMagHeadingSigmaRad;
float nullGain = params::kNullGain;
float holdMinRadS = params::kHoldWheelMinRadS;
float holdMaxPeak = params::kHoldBiasMaxPeakRadS;
float magHard[3] = {params::kMagHardIronUt[0], params::kMagHardIronUt[1], params::kMagHardIronUt[2]};
float imuRoll0Deg = params::kImuMountRollDeg;
float imuPitch0Deg = params::kImuMountPitchDeg;

static uint32_t version_ = 0;

static const Param kParams[] = {
    {"ATT_BW", &attBw, 0.2f, 6.0f, "attitude bandwidth rad/s"},
    {"ATT_ZETA", &attZeta, 0.3f, 2.0f, "attitude damping ratio"},
    {"ATT_KI", &attKi, 0.0f, 0.05f, "attitude Ki N m/(rad s)"},
    {"IXX", &inertia[0], 0.01f, 0.5f, "table inertia x kg m^2"},
    {"IYY", &inertia[1], 0.01f, 0.5f, "table inertia y kg m^2"},
    {"IZZ", &inertia[2], 0.01f, 0.5f, "table inertia z kg m^2"},
    {"SPD_KP", &speedKp, 0.0f, 0.1f, "wheel speed Kp duty/(rad/s)"},
    {"SPD_KI", &speedKi, 0.0f, 1.0f, "wheel speed Ki duty/rad"},
    {"MAG_SIGMA", &magSigma, 0.02f, 3.0f, "mag heading noise rad"},
    {"NULL_GAIN", &nullGain, 0.0f, 20.0f, "null-mode gain 1/s"},
    {"HOLD_MIN", &holdMinRadS, 0.0f, 30.0f, "hold wheel clearance rad/s"},
    {"HOLD_PEAK", &holdMaxPeak, 10.0f, 55.0f, "hold bias peak wheel rad/s"},
    {"MAG_HX", &magHard[0], -100.0f, 100.0f, "mag hard iron x uT"},
    {"MAG_HY", &magHard[1], -100.0f, 100.0f, "mag hard iron y uT"},
    {"MAG_HZ", &magHard[2], -100.0f, 100.0f, "mag hard iron z uT"},
    {"IMU_ROLL0", &imuRoll0Deg, -10.0f, 10.0f, "IMU roll with deck level, deg (then CALIBRATE)"},
    {"IMU_PITCH0", &imuPitch0Deg, -10.0f, 10.0f, "IMU pitch with deck level, deg (then CALIBRATE)"},
};

int count() { return sizeof(kParams) / sizeof(kParams[0]); }
const Param& at(int i) { return kParams[i]; }

const Param* find(const char* name) {
  for (const Param& p : kParams) {
    const char *a = p.name, *b = name;
    while (*a && *b && toupper((unsigned char)*a) == toupper((unsigned char)*b)) {
      a++;
      b++;
    }
    if (!*a && !*b) return &p;
  }
  return nullptr;
}

bool set(const char* name, float value) {
  const Param* p = find(name);
  if (!p) return false;
  *p->value = value < p->lo ? p->lo : (value > p->hi ? p->hi : value);
  version_++;
  return true;
}

uint32_t version() { return version_; }

}  // namespace tune
