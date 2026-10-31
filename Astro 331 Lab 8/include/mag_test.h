#pragma once
#include <stdint.h>

// "MAG TEST": scripted validation of the magnetometer yaw aid (mag_aid.h) on the FIXED table, run
// over the XBee (TODO.md section 0, Owen's test 5 repeated with all four wheels). The table can't
// turn, so the true yaw is the one CALIBRATE defined (0): any yaw the estimate shows is error.
// Procedure: power on, wait ~10 min (gyro warm-up), CALIBRATE 180, then MAG TEST [drift min].
// The wheels follow the schedule below in Manual mode; KICK steps rotate the yaw ESTIMATE at the
// step start (as the KICK command does) to check that the aid pulls it back.
//
// Per step, graded on the EKF yaw (all samples, transitions included):
//   normal step: max |yaw| <= kMagTestTolDeg
//   KICK step:   |yaw| back within kMagTestTolDeg <= kMagTestRecoverS after the kick and never
//                outside it again for the rest of the step
// Also reported: the share of samples the aid applied an update (mode s/S) and the rms field
// residual while it did. After the drift run: max |yaw| and the yaw trend (least squares, deg/h).
//
// Parts (first visits to a direction pattern learn its reference; later visits reuse it):
//   1 stopped: baseline, then KICK +10
//   2 eleven direction patterns at 200 rpm, then repeats at 100 / 300 rpm and mixed speeds
//     (the speed model within a pattern, and stored references being reused)
//   3 KICK -10 and +10 while spinning, then stopped at the new shaft positions
//   4 drift run: kDrift (2 min steps) cycled for the requested time, ending stopped
// KICK steps last 120 s, so the kicked error has decayed (~0.1 deg) before the next pattern's
// reference is learned (the aid's known limit, mag_aid.h).
namespace mag_test {

struct Step {
  int16_t rpm[4];   // A, B, C, D
  uint16_t s;       // duration
  int8_t kickDeg;   // KICK at the step start (0 = none)
};

constexpr Step kMain[] = {
    // 1: stopped
    {{0, 0, 0, 0}, 60, 0},
    {{0, 0, 0, 0}, 120, 10},
    // 2: direction patterns (signs A B C D), first visits
    {{200, 200, 200, 200}, 60, 0},     // ++++
    {{200, -200, 200, -200}, 60, 0},   // +-+-  (the null-mode pattern)
    {{200, 200, -200, -200}, 60, 0},   // ++--
    {{200, -200, -200, 200}, 60, 0},   // +--+
    {{-200, 200, -200, 200}, 60, 0},   // -+-+
    {{-200, -200, -200, -200}, 60, 0}, // ----
    {{200, 200, 200, -200}, 60, 0},    // +++-
    {{-200, 200, 200, -200}, 60, 0},   // -++-
    {{200, -200, -200, -200}, 60, 0},  // +---
    {{-200, -200, 200, 200}, 60, 0},   // --++
    {{-200, 200, 200, 200}, 60, 0},    // -+++
    //    repeats at other speeds (stored references, speed model)
    {{100, 100, 100, 100}, 60, 0},     // ++++
    {{300, -300, 300, -300}, 60, 0},   // +-+-
    {{100, 100, -100, -100}, 60, 0},   // ++--
    {{-300, -300, -300, -300}, 60, 0}, // ----
    {{250, -100, 150, -300}, 60, 0},   // +-+- mixed speeds
    {{0, 0, 0, 0}, 60, 0},             // stopped somewhere new
    // 3: kicks while spinning, then stop
    {{200, -200, 200, -200}, 120, -10},
    {{200, 200, -200, -200}, 120, 10},
    {{0, 0, 0, 0}, 60, 0},
};
constexpr int kNumMain = sizeof(kMain) / sizeof(kMain[0]);

// 4: drift run, 2 min per step, cycled; three new patterns (+-++, ++-+, -+--) among known ones.
constexpr Step kDrift[] = {
    {{200, -200, 200, -200}, 120, 0},  {{0, 0, 0, 0}, 120, 0},          {{150, 150, 150, 150}, 120, 0},
    {{200, -200, 200, 200}, 120, 0},   {{100, -100, 100, -100}, 120, 0}, {{0, 0, 0, 0}, 120, 0},
    {{-300, 300, -300, 300}, 120, 0},  {{200, 200, -200, 200}, 120, 0},  {{-100, -100, -100, -100}, 120, 0},
    {{0, 0, 0, 0}, 120, 0},            {{200, 200, 200, -200}, 120, 0},  {{250, -100, 150, -300}, 120, 0},
    {{-200, 200, -200, -200}, 120, 0}, {{300, 300, -300, -300}, 120, 0}, {{0, 0, 0, 0}, 120, 0},
};
constexpr int kNumDrift = sizeof(kDrift) / sizeof(kDrift[0]);
constexpr float kDriftStepS = 120.0f;

}  // namespace mag_test

// Runs the schedule and grades it. main.cpp sets the wheel speeds and applies the kicks.
class MagTest {
 public:
  enum class Event : uint8_t { None, StepDone, Complete };

  struct Result {
    bool pass = false;
    bool kick = false;
    float maxAbsDeg = 0;   // max |yaw| over the step (KICK steps: after kMagTestRecoverS)
    float peakDeg = 0;     // KICK steps: max |yaw| (the kick itself)
    float endDeg = 0;      // yaw at the end of the step
    float recoverS = 0;    // KICK steps: time until |yaw| stayed within the limit
    float aidFrac = 0;     // share of samples with an applied heading update
    float residRmsDeg = 0; // field residual while applied
  };

  // driftSteps: number of kDrift steps after the main schedule (cycled).
  void start(int driftSteps);
  void abort() { index_ = -1; }
  bool active() const { return index_ >= 0; }
  int index() const { return index_; }  // 0-based, -1 = not running
  int total() const { return mag_test::kNumMain + driftSteps_; }
  bool inDrift() const { return index_ >= mag_test::kNumMain; }
  const mag_test::Step& step() const;

  // Per gyro sample while active. aidInUse: the aid applied an update (mode s/S).
  // On StepDone the result is in lastResult() and index() has moved to the next step;
  // on Complete the drift summary is ready.
  Event update(float dt, float yawDeg, bool aidInUse, float residDeg);

  const Result& lastResult() const { return last_; }
  int passCount() const { return passN_; }
  int doneCount() const { return doneN_; }
  float driftMinutes() const { return driftT_ / 60.0f; }
  float driftMaxAbsDeg() const { return driftMax_; }
  float driftAidFrac() const { return driftN_ ? (float)driftAid_ / driftN_ : 0.0f; }
  float driftTrendDegPerH() const;  // least-squares slope of yaw over the drift run

 private:
  void beginStep();

  int index_ = -1;
  int driftSteps_ = 0;
  float t_ = 0;           // time in the current step
  uint32_t n_ = 0, aidN_ = 0;
  double residSq_ = 0;
  float maxAbs_ = 0, peak_ = 0, lastOutT_ = 0, yaw_ = 0;
  Result last_;
  int passN_ = 0, doneN_ = 0;
  // drift run accumulators (time in s from its start, yaw in deg)
  float driftT_ = 0, driftMax_ = 0;
  uint32_t driftN_ = 0, driftAid_ = 0;
  double sT_ = 0, sY_ = 0, sTT_ = 0, sTY_ = 0;
};
