#pragma once

// Null-mode wheel bias for heading holds (ARM, DEMO LEVEL): keeps every wheel at least minSpeed
// from zero, as far as the wheels' speed budget allows, so the magnetometer aid (mag_aid.h) sees
// a steady direction pattern instead of stopped or slowly reversing shafts, which it has to gate
// (2026-10-30, TODO.md section 0).
//
// With all four wheels, speeds along n = [+1,-1,+1,-1] give no body torque (wheel_geometry.h),
// so the null-mode speed b is free. After moving it to b, wheel i turns at
//   s_i = c_i + n_i b,  |s_i| = |b - t_i|,  t_i = -n_i c_i,
// where c_i is wheel i's speed without its null-mode part. The clearance is min_i |b - t_i| and
// the peak wheel speed max_i |b - t_i|. The four points t_i split the line into five segments
// (below all, between neighbours, above all); each is one direction pattern. Per segment the
// candidate b has clearance >= minSpeed with the lowest peak, or, in a gap narrower than
// 2 minSpeed, the gap's centre. b is always kept where no wheel exceeds maxPeak, so the bias
// never ends a hold early (b = 0, no bias, is in that window whenever the wheels are within it).
// The best segment has the most clearance (up to minSpeed), then the lowest peak.
//
// A fixed bias would let a wheel creep slowly through zero as the yaw drag fills the wheels
// (~75 s at 100 rpm, ~70 s gated, then the aid goes stale). This jumps to the next segment
// through the null mode instead (~1 s, no torque on the table), and once the stored momentum
// keeps all wheels clear by itself the bias goes to 0 (no extra speed).
// Hysteresis: change segment only for kEnterMargin more clearance, or about the same clearance
// (kClearTol) and kSwitchGain lower peak; decisions use speeds low-passed over kSpeedLpfS, and a
// change is held at least kDwellS.
// On the table (DEMO_0003 test 2, 2026-10-31) the first version flipped 15 times in 6 s once wheel B
// reached the peak budget: every segment then had ~7.6 rad/s clearance (a near tie decided by
// 1e-4), and while a change was under way a wheel sat stuck near 0 (static friction), moving the
// decision points. Replaying that log's wheel speeds through this version gives 1 change.
class NullBias {
 public:
  // Motor-level simulation 2026-10-30 (pendulous table, DEMO LEVEL, 420 s): 2 / 3 gave 28 pattern
  // changes and 0.43 deg yaw error; 4 / 8 gives 2 changes, 0.06 deg, any wheel < 4 rad/s 0.4 s in
  // all (115 s with no bias), and the same hold length as no bias.
  static constexpr float kEnterMargin = 4.0f;  // rad/s more clearance to change segment
  static constexpr float kSwitchGain = 8.0f;   // rad/s lower peak speed to change segment
  static constexpr float kClearTol = 1.0f;     // rad/s: clearances this close count as equal
  static constexpr float kDwellS = 5.0f;       // s: minimum time between changes
  static constexpr float kSpeedLpfS = 0.5f;    // s: low-pass on the speeds used to decide

  // Start above all points (b = +minSpeed, pattern A,C + / B,D -) with fresh speed filtering.
  void reset() {
    seg_ = 4;
    lpInit_ = false;
    sinceSwitchS_ = kDwellS;
  }
  // speed: measured wheel speeds (rad/s); dt: s since the last call. Returns the null-mode speed
  // to hold (rad/s).
  float update(const float speed[4], float minSpeed, float maxPeak, float dt);
  int segment() const { return seg_; }
  int switches() const { return switches_; }

 private:
  int seg_ = 4;
  int switches_ = 0;
  float lp_[4] = {0, 0, 0, 0};
  bool lpInit_ = false;
  float sinceSwitchS_ = kDwellS;
};
