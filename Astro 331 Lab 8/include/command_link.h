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
//   WHEEL <A-D> ON|OFF  take a wheel out of service / back in (Idle only). A disabled wheel is
//                  never driven; control allocates over the rest. Encoder faults disable it too.
//   MAG ON | MAG OFF  magnetometer heading aid on (default) / off (gyro-only yaw), mag_aid.h
//   KICK <deg>     test: rotate the yaw ESTIMATE by deg (the aid should pull it back)
//   RAW ON | RAW OFF  USB only in practice: stream one raw row per magnetometer sample (~100 Hz)
//                  instead of the 20 Hz estimator CSV (magnetometer calibration data, see main.cpp)
enum class Cmd : uint8_t { None, Arm, Disarm, Kill, Status, Ping, Test, Speed, Counts, Zero,
                           BeginStatic, DemoSpin, DemoLevel, Stop, Calibrate, MagMap, RawOn, RawOff, Wheel, MagAidOn, MagAidOff, Kick, Unknown };

struct Command {
  static constexpr int kAllWheels = 4;
  Cmd cmd = Cmd::None;
  int wheel = -1;    // Test/Speed: 0..3 for A..D; Speed also accepts kAllWheels
  float value = 0;   // Wheel: 1 on / 0 off. Test: duty. Speed: rpm. BeginStatic: wheel rpm (if hasValue)
  bool hasValue = false;  // BeginStatic: an rpm was given
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
