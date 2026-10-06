#pragma once
#include "frames.h"
#include "quat_math.h"

// Four identical wheels in a pyramid: axes elevated 45 deg above horizontal,
// tilted up toward body +z, horizontal projections at azimuth 0/90/180/270 deg
// (A -> +x, B -> +y, C -> -x, D -> -y; counter-clockwise from above, from +x).
// Positive wheel speed = right-hand rotation about the wheel axis.
//
//   body torque on the table from wheel motor torques u:   tau = -D u
//   wheel dynamics:  J (dw_i + a_i . dOmega) = u_i
namespace wheel_geometry {

constexpr int kN = 4;
constexpr float kAzimuthDeg[kN] = {0.0f, 90.0f, 180.0f, 270.0f};
constexpr float kElevationDeg = 45.0f;

inline Vec3 axis(int i) {
  const float el = kElevationDeg * kDegToRad;
  const float az = kAzimuthDeg[i] * kDegToRad;
  return {cosf(el) * cosf(az), cosf(el) * sinf(az), sinf(el)};
}

// Wheel motor torques (N m, along each axis) that produce the desired body
// torque tauBody, using only the enabled wheels: u = -D+ tau (min-norm).
// Returns false if fewer than 3 independent wheels are enabled.
inline bool allocate(const Vec3& tauBody, const bool enabled[kN], float u[kN]) {
  Mat<3, 3> M;
  for (int i = 0; i < kN; i++) {
    u[i] = 0.0f;
    if (!enabled[i]) continue;
    const Vec3 a = axis(i);
    const float av[3] = {a.x, a.y, a.z};
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) M(r, c) += av[r] * av[c];
  }
  Mat<3, 3> Mi;
  if (!inverse3(M, Mi)) return false;
  const float t[3] = {tauBody.x, tauBody.y, tauBody.z};
  float y[3] = {0, 0, 0};
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) y[r] += Mi(r, c) * t[c];
  for (int i = 0; i < kN; i++) {
    if (!enabled[i]) continue;
    const Vec3 a = axis(i);
    u[i] = -(a.x * y[0] + a.y * y[1] + a.z * y[2]);
  }
  return true;
}

// With all four wheels, D n = 0 for n = [1,-1,1,-1]: speeds along n produce no
// body torque. Adds a torque that holds that mode at `bias` rad/s (0 = nulled).
inline void addNullSpaceTorque(const float wheelSpeed[kN], float wheelInertia, float gain,
                               float bias, float u[kN]) {
  const float n[kN] = {1.0f, -1.0f, 1.0f, -1.0f};
  const float s = 0.25f * (wheelSpeed[0] - wheelSpeed[1] + wheelSpeed[2] - wheelSpeed[3]);
  for (int i = 0; i < kN; i++) u[i] += wheelInertia * gain * (bias - s) * n[i];
}

}  // namespace wheel_geometry
