#include "null_bias.h"

#include <math.h>

float NullBias::update(const float speedIn[4], float minSpeed, float maxPeak, float dt) {
  static const float n[4] = {1.0f, -1.0f, 1.0f, -1.0f};
  // Decide on low-passed speeds: a pattern change drives wheels through zero, where static
  // friction holds them for a moment; raw speeds then move the decision points and the choice
  // flipped back and forth (DEMO_0003 test 2: 15 changes in 6 s).
  const float a = lpInit_ ? dt / (kSpeedLpfS + dt) : 1.0f;
  for (int i = 0; i < 4; i++) lp_[i] += a * (speedIn[i] - lp_[i]);
  lpInit_ = true;
  sinceSwitchS_ += dt;
  const float* speed = lp_;
  const float sNull = 0.25f * (speed[0] - speed[1] + speed[2] - speed[3]);
  float t[4];
  for (int i = 0; i < 4; i++) t[i] = sNull - n[i] * speed[i];  // = -n_i c_i
  // sort ascending (4 elements)
  for (int i = 1; i < 4; i++)
    for (int j = i; j > 0 && t[j] < t[j - 1]; j--) {
      const float x = t[j];
      t[j] = t[j - 1];
      t[j - 1] = x;
    }
  const float mid = 0.5f * (t[0] + t[3]);
  // Peak-speed window: |b - t_i| <= maxPeak for all i. Empty => the wheels are past the budget
  // anyway; use the lowest peak.
  float wLo = t[3] - maxPeak, wHi = t[0] + maxPeak;
  if (wLo > wHi) wLo = wHi = mid;

  float b[5], clear[5], peak[5];
  for (int k = 0; k <= 4; k++) {
    float x;
    if (k == 0) {
      x = t[0] - minSpeed;
    } else if (k == 4) {
      x = t[3] + minSpeed;
    } else {
      const float lo = t[k - 1] + minSpeed, hi = t[k] - minSpeed;
      // gap wide enough: lowest peak inside it; else its centre (the most clearance it has)
      x = lo <= hi ? (mid < lo ? lo : (mid > hi ? hi : mid)) : 0.5f * (t[k - 1] + t[k]);
    }
    if (x < wLo) x = wLo;
    if (x > wHi) x = wHi;
    b[k] = x;
    float c = 1e9f;
    for (int i = 0; i < 4; i++) c = fminf(c, fabsf(x - t[i]));
    clear[k] = fminf(c, minSpeed);  // more clearance than minSpeed doesn't count
    peak[k] = fmaxf(x - t[0], t[3] - x);
  }
  // Best: most clearance, then lowest peak ("same" clearance within kClearTol; ties between the
  // outer segments keep the current one).
  int best = seg_;
  for (int k = 0; k <= 4; k++) {
    if (clear[k] > clear[best] + kClearTol ||
        (clear[k] > clear[best] - kClearTol && peak[k] < peak[best] - 1e-4f))
      best = k;
  }
  // Hysteresis: change only for kEnterMargin more clearance, or about the same clearance and
  // kSwitchGain lower peak speed, and never within kDwellS of the last change (let the wheels
  // finish the move first).
  if (best != seg_ && sinceSwitchS_ >= kDwellS &&
      (clear[best] > clear[seg_] + kEnterMargin ||
       (clear[best] > clear[seg_] - kClearTol && peak[best] < peak[seg_] - kSwitchGain))) {
    seg_ = best;
    switches_++;
    sinceSwitchS_ = 0.0f;
  }
  return b[seg_];
}
