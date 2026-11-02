#pragma once
#include <stdint.h>

// "DEMO ALL [hold s] [A-D]": the whole planned demonstration as one continuous armed run, so nothing
// is disarmed (and no wheel is braked) between the parts. main.cpp drives the existing modules; this
// class keeps the sequence and grades part 1.
//   1 Hold   DEMO LEVEL (demo_spin.h): heading held at the start value, roll/pitch damped at the rest
//            tilt. Once it has settled, holdS seconds are graded: requirement 1 = the heading stays
//            within +/-kFullHoldTolDeg of the start heading (estimate; the laser is the truth), with its
//            jitter (rms about the mean) and drift (end mean - start mean) reported.
//   2 Slew   SLEW YAW (slew_test.h) handed over without a reset: same 4 targets, yaw relative to the
//            held start heading; requirement 2 / stretch. It ends holding the start heading.
//   3 Fault  FAULT TEST on wheel X (wheel_fault.h) while that hold continues: the motor fails
//            kFaultDelayS after the slews (time to read the laser), requirement 3.
//   4 Done   summary of all three; the table keeps holding until STOP (or KILL).
// A disarm (STOP, KILL, safety, wheel limit) at any point ends it and says where.
class FullDemo {
 public:
  enum class Phase : uint8_t { Off, Hold, Slew, Fault, Done };

  void start(float holdS, int faultWheel);
  void abort() { phase_ = Phase::Off; }
  void setPhase(Phase p) { phase_ = p; }

  bool active() const { return phase_ != Phase::Off; }
  Phase phase() const { return phase_; }
  char phaseChar() const;  // - off, 1 hold, 2 slews, 3 fault test, D done (holding)
  int part() const { return static_cast<int>(phase_); }  // 1..3 while running, 4 done
  int faultWheel() const { return wheel_; }
  float holdS() const { return holdS_; }

  // Part 1, every control step once DEMO LEVEL has settled: heading error from the start heading
  // (deg). Returns true on the step the graded hold is complete.
  bool holdStep(float dt, float yawErrDeg);
  float holdMaxErrDeg() const { return maxErr_; }
  float holdJitterDeg() const;  // rms about the mean over the whole graded hold
  float holdDriftDeg() const;   // mean of the last kFullDriftWinS - mean of the first
  bool holdPass() const;

 private:
  Phase phase_ = Phase::Off;
  float holdS_ = 60.0f;
  int wheel_ = 1;
  float t_ = 0.0f, maxErr_ = 0.0f;
  double n_ = 0, sum_ = 0, sq_ = 0;
  double n0_ = 0, sum0_ = 0, n1_ = 0, sum1_ = 0;  // first / last kFullDriftWinS
};
