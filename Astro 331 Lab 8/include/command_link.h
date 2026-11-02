#pragma once
#include <Arduino.h>

// Line-oriented ASCII command parser over any Stream (XBee Serial8 and USB Serial
// both use this). Commands (case-insensitive, newline terminated):
//   ARM            hold level, yaw 0
//   DISARM | KILL  stop all wheels immediately (KILL is also accepted as "K")
//   STATUS         one-line state report
//   PING           reply "PONG"
//   TEST <A-D> <duty>   bench: spin one wheel open-loop at duty (-0.5..0.5) for 1 s (Idle only)
//   SPEED <A-D|ALL> <rpm>  manual: closed-loop wheel speed setpoint (from Idle or Manual;
//                          enters Manual). DISARM/KILL stops all wheels and returns to Idle.
//   COUNTS         raw encoder counts per wheel since the last ZERO (+ output turns at kEncoderCountsPerRev)
//   ZERO           zero the COUNTS readout (any state; does not affect speed or control)
//   BEGIN STATIC TEST [rpm]  start logging every row to a new STATIC_####.CSV on the SD card
//                      until power is removed, and hold all four wheels at a constant speed
//                      (default kStaticTestWheelRpm = 100; "0" = motors off) via Manual mode.
//                      Refused while calibrating; a repeat just reports the file.
//   DEMO SPIN      demo: level from <=30 deg tilt (keeping heading), spin up CCW about +z over
//                  10 s to the highest rate the wheels' momentum allows (<= 30 rpm), hold 10 s,
//                  spin down over 10 s, settle 3 s, then Idle. From Idle only, wheels stopped.
//   DEMO LEVEL     level-and-hold test: same leveling as DEMO SPIN (keeps heading), then hold
//                  level indefinitely with no spin and no wheel bias, until STOP / KILL / a
//                  wheel reaches kDemoWheelAbortRadS. Reports when level is reached and how
//                  long it was held. From Idle only, wheels stopped; logs DEMO_####.CSV.
//   STOP           controlled end of the demo (spin down from the current rate); KILL is the
//                  emergency stop (brakes the wheels, which also stops the table abruptly)
//   CALIBRATE [s]  redo the static gyro-bias calibration (Idle only; keep still). Optional
//                  duration 10-900 s (default kCalDurationMs). Best after ~10 min of warm-up.
//   MAG MAP        magnetometer disturbance map: step the wheels through the schedule in
//                  mag_map.h (~11 min) while logging to MAGMAP_####.CSV; table must be FIXED.
//                  STOP / DISARM / KILL end it (wheels stop, logging continues).
//   MAG TEST [min] yaw-aid validation on the FIXED table (mag_test.h): ~25 min of stopped /
//                  direction-pattern / KICK steps, then a drift run of min minutes (default
//                  kMagTestDriftMin = 30; 0 skips it). PASS/FAIL per step on |yaw| <= 1 deg; logs
//                  MAGTEST_####.CSV. STOP / DISARM / KILL end it. Best after warm-up + CALIBRATE 180.
//   WHEEL <A-D> ON|OFF  take a wheel out of service / back in (Idle only). A disabled wheel is
//                  never driven; control allocates over the rest. Encoder faults disable it too.
//   MAG ON | MAG OFF  magnetometer heading aid on (default) / off (gyro-only yaw), mag_aid.h
//   KICK <deg>     test: rotate the yaw ESTIMATE by deg (the aid should pull it back)
//   SET <name> <value>  change a runtime-tunable parameter (tune.h: ATT_BW, ATT_ZETA, ATT_KI, IXX,
//                  IYY, IZZ, SPD_KP, SPD_KI, MAG_SIGMA, NULL_GAIN, HOLD_MIN, HOLD_PEAK, MAG_HX/HY/HZ);
//                  any state, clamped to a safe range, lost at power-off.  GET [name]  print one / all.
//   LEVEL ZERO     deck LEVEL (spirit level) and clamped, Idle: measure the IMU mounting tilt (5 s),
//                  rotate all IMU data into the table frame, then CALIBRATE. SET IMU_ROLL0/PITCH0.
//   STEP TEST [deg]  air bearing, from Idle: DEMO LEVEL, then heading steps +A, 0, -A, 0 (default 5 deg,
//                  20 s each) graded on rise / overshoot / settling / error / jitter, plus the yaw
//                  inertia from momentum exchange (step_test.h). STOP / KILL end it.
//   BIAS <rpm>     heading holds (ARM, DEMO LEVEL): keep every wheel at least rpm from zero with
//                  the null mode (null_bias.h; default 100, 0 = off). Any state; next hold uses it.
//   SLEW TEST      stretch requirement: slew to 4 attitudes (shortest rotation, smooth profile), reach
//                  each within +/-1 deg in roll/pitch/yaw and hold 5 s; reports PASS/FAIL per target.
//                  From Idle, wheels stopped; logs SLEW_####.CSV. After target 4 it keeps holding it
//                  until STOP (or KILL). Tilt kept under 25 deg while it runs.
//   SLEW YAW       stretch requirement as this pendulous table can do it: the 4 targets' yaws only,
//                  roll/pitch left at the rest tilt (damped, as DEMO LEVEL). Graded like SLEW TEST,
//                  roll/pitch from the rest tilt. Idle only; STOP ends it.
//   SLEW TILT      tilted targets on a table balanced near neutral (slew_test.h): 360 deg rest-tilt
//                  scan, then each target = rest tilt + (roll, pitch) offset at its heading, with
//                  momentum unloading between targets. Needs SET GRAV_K. Idle only; STOP ends it.
//   SLEW TSET <1-4> <droll> <dpitch> <yaw>  change a SLEW TILT target (offsets from the rest tilt, deg).
//   SLEW SET <1-4> <roll> <pitch> <yaw>  change a target (deg; yaw relative to the start heading;
//                  tilt <= kSlewMaxTiltDeg). Idle only.  SLEW LIST  print the targets.
//   FAULT TEST <A-D> [s]  requirement 3: after s seconds Armed (default kFaultDelayS = 10) wheel X's
//                  motor goes dead without telling the controller (WheelMotor::setSimulatedDead);
//                  whatever is running carries on. Grades detection and recovery (wheel_fault.h).
//                  Send it from Idle (then start ARM / DEMO LEVEL / SLEW TEST) or while Armed. The
//                  wheels idle at +/-100 rpm in the null mode while it runs.
//   FAULT TEST REAL  same grading, nothing injected: remove power from one wheel yourself.
//   FAULT CLEAR    Idle only: end the test, undo the simulated failure and put wheels the
//                  dead-motor detector took out back in service.
//   RAW ON | RAW OFF  USB only in practice: stream one raw row per magnetometer sample (~100 Hz)
//                  instead of the 20 Hz estimator CSV (magnetometer calibration data, see main.cpp)
enum class Cmd : uint8_t { None, Arm, Disarm, Kill, Status, Ping, Test, Speed, Counts, Zero,
                           BeginStatic, DemoSpin, DemoLevel, Stop, Calibrate, MagMap, MagTest, RawOn, RawOff, Wheel, MagAidOn, MagAidOff, Kick, Bias, Set, Get, StepTest, LevelZero, SlewTest, SlewYaw, SlewTilt, SlewSet, SlewTSet, SlewList, FaultTest, FaultClear, Unknown };

struct Command {
  static constexpr int kAllWheels = 4;
  Cmd cmd = Cmd::None;
  int wheel = -1;    // Test/Speed: 0..3 for A..D; Speed also accepts kAllWheels
  float value = 0;   // Wheel: 1 on / 0 off. Test: duty. Speed: rpm. BeginStatic: wheel rpm (if hasValue)
                     // MagTest: drift-run minutes (if hasValue)
  bool hasValue = false;  // BeginStatic: an rpm was given
  float v[3] = {0, 0, 0};  // SlewSet: roll, pitch, yaw (deg); wheel = target index 0..3
  char name[16] = "";      // Set/Get: parameter name (tune.h)
  // FaultTest: wheel 0..3 (-1 = REAL), value = delay (s) if hasValue
};

class CommandLink {
 public:
  explicit CommandLink(Stream& s) : s_(s) {}

  // Drains available bytes. Returns true (and fills `out`) when a full line was parsed.
  // Call repeatedly until it returns false.
  bool poll(Command& out);

  // Non-blocking send: drops the message if the TX buffer lacks room.
  bool send(const char* msg);

 private:
  Stream& s_;
  char buf_[40];
  uint8_t len_ = 0;
  bool overflow_ = false;
};
