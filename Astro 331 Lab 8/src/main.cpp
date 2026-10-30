#include <Arduino.h>

#include "attitude_ctrl.h"
#include "attitude_ekf.h"
#include "command_link.h"
#include "control_params.h"
#include "data_logger.h"
#include "demo_spin.h"
#include "slew_test.h"
#include "frames.h"
#include "imu_bno085.h"
#include "mag_aid.h"
#include "mag_map.h"
#include "pins.h"
#include "wheel_fault.h"
#include "wheel_geometry.h"
#include "wheel_motor.h"
#include "wheel_speed_ctrl.h"

// Reaction-wheel attitude hold (level, yaw 0) on a BNO085 gyro+accel MEKF.
// States: Calibrating (hold still, ~3 s) -> Idle (wheels off) -> Armed (hold).
// Commands arrive over the XBee (Serial8) and, for bench use, USB Serial.
// Sensors and control run at 100 Hz. One data row per gyro sample (~100 Hz) streams to the
// XBee (every kXbeeLogDecimation-th row) and, after "BEGIN STATIC TEST", to an SD file until
// power is removed (data_logger.h). No SD file is created at boot.
// "DEMO SPIN" runs the level-then-spin demonstration (demo_spin.h) in the Armed state.

using namespace wheel_geometry;

static Bno085Imu imu(kPinImuReset, kPinImuInt);
static AttitudeEKF ekf;
static AttitudeController ctrl;
static CommandLink xbee(XBEE_SERIAL);
static CommandLink usb(Serial);
static DataLogger logger(XBEE_SERIAL);
static DemoSpin demo;
static SlewTest slew;
static MagAid magAid;  // magnetometer heading aid (mag_aid.h)
static uint32_t lastMagUs = 0;
static bool haveLastMag = false;
static MagResult lastMagResult = MagResult::Gated;

// Serial8's built-in TX buffer is only 40 bytes; without this, any line longer than
// that (data rows, STATUS replies) is always dropped by the non-blocking senders.
static uint8_t xbeeTxExtra[1024];

static WheelMotor wheels[kN] = {
    WheelMotor(kPinWheelPwm1[0], kPinWheelPwm2[0], kPinWheelEncA[0], kPinWheelEncB[0],
               params::kMotorSign[0], params::kEncoderSign[0]),
    WheelMotor(kPinWheelPwm1[1], kPinWheelPwm2[1], kPinWheelEncA[1], kPinWheelEncB[1],
               params::kMotorSign[1], params::kEncoderSign[1]),
    WheelMotor(kPinWheelPwm1[2], kPinWheelPwm2[2], kPinWheelEncA[2], kPinWheelEncB[2],
               params::kMotorSign[2], params::kEncoderSign[2]),
    WheelMotor(kPinWheelPwm1[3], kPinWheelPwm2[3], kPinWheelEncA[3], kPinWheelEncB[3],
               params::kMotorSign[3], params::kEncoderSign[3]),
};
static WheelSpeedLoop speedLoops[kN];
static bool wheelEnabled[kN] = {true, true, true, true};  // fault-handling hook
// Encoder fault detection (wheelLoop): a wheel driven hard that never shows speed, or whose
// speed jumps faster than the motor could accelerate it, is disabled (wheelEnabled = false).
// 2026-10-27: wheel D's encoder lost a channel; D spun at ~500 rpm reading 0 and glitched to
// -165 rad/s while still, which disarmed and even drove D through its own speed loop.
static float encFaultS[kN] = {0, 0, 0, 0};
static float prevSpeed[kN] = {0, 0, 0, 0};
// Dead-motor detection (requirement 3, wheel_fault.h): a wheel driven hard that doesn't accelerate
// is disabled and then coasts (wheelDead). FAULT TEST injects such a failure and grades recovery.
static DeadMotorDetector deadDet[kN];
static bool wheelDead[kN] = {false, false, false, false};
static FaultTest fault;
static float wheelCmd[kN] = {0, 0, 0, 0};                 // wheel speed setpoints, rad/s
static float manualTarget[kN] = {0, 0, 0, 0};             // SPEED targets; wheelCmd ramps to these
static float manualSpeed[kN] = {0, 0, 0, 0};              // low-passed wheel speed, rad/s (Manual)

// Manual: each wheel's speed loop tracks a setpoint typed with SPEED (bench / spin-up).
enum class State : uint8_t { Calibrating, Idle, Armed, Manual };
static State state = State::Calibrating;
static char stateChar() {
  switch (state) {
    case State::Calibrating: return 'C';
    case State::Idle: return 'I';
    case State::Armed: return 'A';
    default: return 'M';
  }
}

// ---- Static calibration accumulators ----
static constexpr float kCalMaxGyroStd = 0.01f;    // rad/s per axis, else "not still"
static constexpr float kCalMaxGyroMean = 0.05f;   // rad/s, rejects a rotating table
static constexpr uint32_t kMinCalSamples = 100;

static uint32_t calStartMs = 0, calProgressMs = 0;
static uint32_t calDurationMs = params::kCalDurationMs;  // boot default; CALIBRATE <s> overrides
static uint32_t calGyroN = 0, calAccelN = 0;
static double gSum[3], gSumSq[3], aSum[3];

// ---- IMU sample-loss diagnostics (gyro stream, all states) ----
// seqMissed: reports the BNO085 sent but the Teensy never received (gaps in its sequence number).
// chipGaps:  intervals > kImuGapMs with NO sequence gap, i.e. the BNO085 itself sent late.
static uint8_t lastGyroSeq = 0;
static uint32_t lastGyroSeqUs = 0;
static bool haveGyroSeq = false;
static uint32_t seqMissed = 0, chipGaps = 0;
static float maxGyroGapMs = 0.0f;

static uint32_t lastGyroUs = 0, lastGyroMs = 0;
static bool haveLastGyro = false;
static uint32_t lastUsbMs = 0, lastWheelUs = 0;
// RAW ON: USB carries one raw row per magnetometer sample instead of the estimator CSV.
static bool usbRaw = false;
// 64-bit sample clock for log timestamps (micros() wraps every ~71.6 min).
static uint64_t logClockUs = 0;
static uint32_t logLastRawUs = 0;
static bool logClockStarted = false;
static AccelResult lastAccelResult = AccelResult::Applied;

// Latest raw sensor values, for the data log (updated in every state).
static Vec3 latestGyro{0, 0, 0}, latestAccel{0, 0, 0}, latestMag{0, 0, 0};

// Bench wheel test (Idle only)
static int testWheel = -1;
static float testDuty = 0.0f;
static uint32_t testEndMs = 0;

static void sayAll(const char* msg) {
  xbee.send(msg);
  usb.send(msg);
}

static void stopAllWheels() {
  for (int i = 0; i < kN; i++) wheels[i].stop();
}

// MAG MAP progress: current schedule step (-1 = not running) and when it started.
static int magStep = -1;
static uint32_t magStepStartMs = 0;

static void disarm(const char* reason) {
  stopAllWheels();
  testWheel = -1;
  demo.abort();
  slew.abort();
  magStep = -1;
  if (state == State::Armed || state == State::Manual) state = State::Idle;
  char msg[48];
  snprintf(msg, sizeof(msg), "# DISARM %s\n", reason);
  sayAll(msg);
  if (fault.active()) {
    fault.abort();
    sayAll("# FAULT TEST ABORTED (disarmed)\n");
  }
}

// ---- Wheel failure (requirement 3) ----
static char wheelChar(int i) { return i < 0 ? '?' : (char)('A' + i); }

// 4-char wheel status for STATUS and the log: '.' in service, 'x' in service but its motor is
// failed by FAULT TEST (not detected yet), 'D' dead motor detected (out of service, coasting),
// '-' out of service (WHEEL x OFF or encoder fault).
static void wheelStatus(char out[kN + 1]) {
  for (int i = 0; i < kN; i++)
    out[i] = wheelEnabled[i] ? (wheels[i].simulatedDead() ? 'x' : '.') : (wheelDead[i] ? 'D' : '-');
  out[kN] = '\0';
}

static void reportFault(FaultTest::Event ev) {
  char msg[200];
  const Vec3 pk = fault.peakErrDeg(), hm = fault.holdMaxDeg();
  switch (ev) {
    case FaultTest::Event::Inject:
      wheels[fault.wheel()].setSimulatedDead(true);
      snprintf(msg, sizeof(msg), "# FAULT TEST: wheel %c motor now DEAD (simulated; controller not told)\n",
               wheelChar(fault.wheel()));
      break;
    case FaultTest::Event::Recovered:
      snprintf(msg, sizeof(msg),
               "# FAULT TEST: back within +/-%.1f deg %.1f s after the %s, peak err %.2f %.2f %.2f deg; holding %.0f s\n",
               params::kFaultTolDeg, fault.recoverS(), fault.real() ? "detection" : "failure", pk.x, pk.y, pk.z,
               params::kFaultHoldS);
      break;
    case FaultTest::Event::Pass:
    case FaultTest::Event::Fail: {
      char det[24] = "n/a (REAL)";
      if (!fault.real()) snprintf(det, sizeof(det), "%.2f s", fault.detectS());
      snprintf(msg, sizeof(msg),
               "# FAULT TEST %s%s%s: wheel %c, detected %s, recovered %.1f s, peak err %.2f %.2f %.2f, hold max "
               "%.2f %.2f %.2f deg\n",
               fault.pass() ? "PASS" : "FAIL", fault.pass() ? "" : " - ", fault.failReason(),
               wheelChar(fault.wheel()), det, fault.recoverS(), pk.x, pk.y, pk.z, hm.x, hm.y, hm.z);
      break;
    }
    default:
      return;
  }
  sayAll(msg);
}

// A detector took wheel i out of service: tell the fault test.
static void onWheelFailed(int i) {
  const FaultTest::Event ev = fault.onFailure(i);
  if (ev != FaultTest::Event::None) reportFault(ev);
}

static void startFaultTest(const Command& c, CommandLink& from) {
  if (state != State::Idle && state != State::Armed) {
    from.send("# FAULT TEST REFUSED: Idle or Armed only\n");
    return;
  }
  if (fault.active()) {
    from.send("# FAULT TEST REFUSED: already running\n");
    return;
  }
  if (c.wheel >= 0 && (!wheelEnabled[c.wheel] || wheels[c.wheel].simulatedDead())) {
    from.send("# FAULT TEST REFUSED: that wheel is already out of service\n");
    return;
  }
  int n = 0;
  for (int i = 0; i < kN; i++) n += wheelEnabled[i] ? 1 : 0;
  const float delay = c.hasValue ? c.value : params::kFaultDelayS;
  fault.start(c.wheel, delay);
  if (n < kN) {
    fault.setWheelBias(0.0f);  // the null mode needs all four wheels
    sayAll("# FAULT TEST WARNING: not all 4 wheels in service - no wheel bias; one more failure may leave too few\n");
  }
  char msg[128];
  if (fault.real()) {
    snprintf(msg, sizeof(msg), "# FAULT TEST REAL: watching for a dead motor - remove power from one wheel%s\n",
             state == State::Armed ? " now" : " once Armed");
  } else {
    snprintf(msg, sizeof(msg), "# FAULT TEST: wheel %c motor fails after %.0f s Armed%s\n", 'A' + c.wheel, delay,
             state == State::Armed ? "" : " (now start ARM / DEMO LEVEL / SLEW TEST)");
  }
  sayAll(msg);
}

static void clearFaults(CommandLink& from) {
  if (state != State::Idle) {
    from.send("# FAULT CLEAR REFUSED: not idle\n");
    return;
  }
  fault.abort();
  for (int i = 0; i < kN; i++) {
    wheels[i].setSimulatedDead(false);
    if (wheelDead[i]) wheelEnabled[i] = true;
    wheelDead[i] = false;
    deadDet[i].reset();
    encFaultS[i] = 0.0f;
    prevSpeed[i] = wheels[i].speed();
  }
  char ws[kN + 1];
  wheelStatus(ws);
  char msg[64];
  snprintf(msg, sizeof(msg), "# FAULT CLEAR: wheels %s\n", ws);
  sayAll(msg);
}

static void arm(CommandLink& from) {
  if (state != State::Idle) {
    from.send("# ARM REFUSED: not idle\n");
    return;
  }
  const Vec3 e = ekf.eulerRad();
  const float lim = params::kMaxTiltDeg * kDegToRad;
  if (fabsf(e.x) > lim || fabsf(e.y) > lim) {
    from.send("# ARM REFUSED: tilt too large\n");
    return;
  }
  ctrl.reset();
  ctrl.setReference({1.0f, 0.0f, 0.0f, 0.0f});  // level, yaw 0
  for (int i = 0; i < kN; i++) {
    speedLoops[i].reset();
    wheelCmd[i] = wheels[i].speed();
  }
  state = State::Armed;
  sayAll("# ARMED\n");
}

// Total wheel angular momentum in the body frame: h = sum a_i J (w_i + a_i . omega).
static Vec3 wheelMomentum(const Vec3& omega) {
  Vec3 h{0, 0, 0};
  for (int i = 0; i < kN; i++) {
    const Vec3 a = axis(i);
    h = h + a * (params::kWheelInertia * (wheels[i].speed() + dot(a, omega)));
  }
  return h;
}

// Highest demo yaw rate the wheels can absorb: margin * sum(a_i.z) J w_max / I_zz, capped at
// the requested 30 rpm. Only enabled wheels count.
static float demoTargetRate() {
  float az = 0.0f;
  bool all = true;
  for (int i = 0; i < kN; i++) {
    if (wheelEnabled[i]) az += fabsf(axis(i).z);
    all = all && wheelEnabled[i];
  }
  // The null-mode wheel bias (all four wheels only) uses part of each wheel's speed range.
  const float usable = params::kWheelMaxSpeedRadS - (all ? params::kDemoWheelBiasRadS : 0.0f);
  const float cap = params::kDemoMomentumMargin * az * params::kWheelInertia * usable / params::kBodyInertia[2];
  return cap < params::kDemoRequestedRateRadS ? cap : params::kDemoRequestedRateRadS;
}

// DEMO LEVEL summary: total time armed and how long level (within tolerance) was held.
static void reportLevelHold(const char* why) {
  char msg[112];
  const Vec3 e = ekf.eulerRad() * kRadToDeg;
  snprintf(msg, sizeof(msg), "# %s: armed %.1f s, rest tilt held %.1f s, roll %.2f pitch %.2f deg\n", why,
           demo.elapsed(), demo.levelHoldTime(), e.x, e.y);
  sayAll(msg);
}

// levelOnly: DEMO LEVEL (level and hold, no spin). Otherwise DEMO SPIN.
static void startDemo(CommandLink& from, bool levelOnly) {
  if (state != State::Idle) {
    from.send("# DEMO REFUSED: not idle (wait for calibration / DISARM first)\n");
    return;
  }
  const Vec3 e = ekf.eulerRad();
  const float lim = params::kMaxTiltDeg * kDegToRad;
  if (fabsf(e.x) > lim || fabsf(e.y) > lim) {
    from.send("# DEMO REFUSED: tilt over 30 deg\n");
    return;
  }
  for (int i = 0; i < kN; i++) {
    if (fabsf(wheels[i].speed()) > params::kDemoStartMaxWheelRadS) {
      from.send("# DEMO REFUSED: wheels still turning\n");
      return;
    }
  }
  if (!logger.sdActive()) logger.startFile("DEMO");  // record the run if nothing is logging yet

  const float rate = demoTargetRate();
  ctrl.reset();
  demo.start(ekf.quaternion(), rate, levelOnly);
  ctrl.setTrajectory(demo.ref().q, demo.ref().w, demo.ref().a);
  for (int i = 0; i < kN; i++) {
    speedLoops[i].reset();
    wheelCmd[i] = wheels[i].speed();
  }
  state = State::Armed;
  char msg[128];
  if (levelOnly) {
    snprintf(msg, sizeof(msg), "# DEMO LEVEL START: holding the natural rest tilt (STOP to end)%s%s\n",
             logger.sdActive() ? ", log " : "", logger.sdActive() ? logger.fileName() : "");
  } else {
    snprintf(msg, sizeof(msg), "# DEMO START: leveling, then spin CCW to %.2f rad/s (%.1f rpm)%s%s\n", rate,
             rate * 60.0f / 6.2831853f, logger.sdActive() ? ", log " : "",
             logger.sdActive() ? logger.fileName() : "");
  }
  sayAll(msg);
}

// ---- SLEW TEST (stretch requirement) ----
static void sayTarget(const char* head, int i) {
  const SlewTest::Target& g = slew.target(i);
  char msg[112];
  snprintf(msg, sizeof(msg), "# %s %d: roll %.1f pitch %.1f yaw %+.1f deg (yaw rel. to start)\n", head, i + 1,
           g.rollDeg, g.pitchDeg, g.yawDeg);
  sayAll(msg);
}

static void reportSlewResult(int i) {
  const SlewTest::Result& r = slew.result(i);
  char msg[128];
  snprintf(msg, sizeof(msg), "# SLEW %d/%d %s: %s, max err roll %.2f pitch %.2f yaw %.2f deg over %.0f s\n", i + 1,
           params::kSlewNumTargets, r.pass ? "PASS" : "FAIL", r.settled ? "settled" : "NOT settled",
           r.maxErrDeg.x, r.maxErrDeg.y, r.maxErrDeg.z, params::kSlewHoldS);
  sayAll(msg);
}

static void sayNextSlew() {
  char msg[112];
  snprintf(msg, sizeof(msg), "# SLEW %d/%d: %.1f deg in %.1f s\n", slew.index() + 1, params::kSlewNumTargets,
           slew.slewAngleDeg(), slew.slewDuration());
  sayAll(msg);
}

static void startSlew(CommandLink& from) {
  if (state != State::Idle || demo.active()) {
    from.send("# SLEW REFUSED: not idle (wait for calibration / DISARM first)\n");
    return;
  }
  const Vec3 e = ekf.eulerRad();
  const float lim = params::kMaxTiltDeg * kDegToRad;
  if (fabsf(e.x) > lim || fabsf(e.y) > lim) {
    from.send("# SLEW REFUSED: tilt over 30 deg\n");
    return;
  }
  for (int i = 0; i < kN; i++) {
    if (fabsf(wheels[i].speed()) > params::kDemoStartMaxWheelRadS) {
      from.send("# SLEW REFUSED: wheels still turning\n");
      return;
    }
  }
  if (!logger.sdActive()) logger.startFile("SLEW");
  ctrl.reset();
  slew.start(ekf.quaternion());
  ctrl.setTrajectory(slew.refQ(), slew.refW(), slew.refA());
  for (int i = 0; i < kN; i++) {
    speedLoops[i].reset();
    wheelCmd[i] = wheels[i].speed();
  }
  state = State::Armed;
  char msg[96];
  snprintf(msg, sizeof(msg), "# SLEW TEST START: %d targets, +/-%.1f deg, hold %.0f s%s%s\n", params::kSlewNumTargets,
           params::kSlewTolDeg, params::kSlewHoldS, logger.sdActive() ? ", log " : "",
           logger.sdActive() ? logger.fileName() : "");
  sayAll(msg);
  for (int i = 0; i < params::kSlewNumTargets; i++) sayTarget("SLEW target", i);
  sayNextSlew();
}

// Called every control step while the slew test runs. Returns false if it disarmed.
static bool stepSlew(float dt) {
  switch (slew.step(dt, ekf.quaternion())) {
    case SlewTest::Event::Settled: {
      char msg[80];
      snprintf(msg, sizeof(msg), "# SLEW %d/%d reached (+/-%.1f deg) %.1f s after the slew, holding\n",
               slew.index() + 1, params::kSlewNumTargets, params::kSlewTolDeg, slew.result(slew.index()).settleS);
      sayAll(msg);
      break;
    }
    case SlewTest::Event::SettleTimeout: {
      const Vec3 e = slew.errorDeg();
      char msg[112];
      snprintf(msg, sizeof(msg), "# SLEW %d/%d NOT reached in %.0f s (err %.2f %.2f %.2f deg), holding anyway\n",
               slew.index() + 1, params::kSlewNumTargets, params::kSlewSettleTimeoutS, e.x, e.y, e.z);
      sayAll(msg);
      break;
    }
    case SlewTest::Event::TargetDone:
      reportSlewResult(slew.index() - 1);
      sayNextSlew();
      break;
    case SlewTest::Event::Complete: {
      reportSlewResult(params::kSlewNumTargets - 1);
      char msg[96];
      snprintf(msg, sizeof(msg), "# SLEW TEST COMPLETE: %d/%d PASS, holding target %d until STOP\n",
               slew.passCount(), params::kSlewNumTargets, params::kSlewNumTargets);
      sayAll(msg);
      break;
    }
    default:
      break;
  }
  float wmax = 0.0f;
  for (int i = 0; i < kN; i++) wmax = fmaxf(wmax, fabsf(wheels[i].speed()));
  if (wmax > params::kDemoWheelAbortRadS) {
    char msg[96];
    snprintf(msg, sizeof(msg), "# SLEW %d/%d: wheel at %.0f rad/s (limit), stopping after %.1f s\n",
             slew.index() + 1, params::kSlewNumTargets, wmax, slew.elapsed());
    sayAll(msg);
    disarm("SLEW_WHEEL_LIMIT");
    return false;
  }
  ctrl.setTrajectory(slew.refQ(), slew.refW(), slew.refA());
  return true;
}

static void stopDemo(CommandLink& from) {
  if (!demo.active()) {
    from.send("# STOP: no demo running\n");
    return;
  }
  if (demo.levelOnly()) reportLevelHold("DEMO LEVEL STOPPED");
  switch (demo.requestStop()) {
    case DemoSpin::Event::Complete:
      disarm("DEMO_STOPPED");  // stopped while leveling: nothing is spinning
      break;
    case DemoSpin::Event::SpinDownStarted:
      sayAll("# DEMO STOP: spinning down\n");
      break;
    default:
      from.send("# STOP: already spinning down\n");
      break;
  }
}

static void resetCalibration() {
  calStartMs = calProgressMs = millis();
  calGyroN = calAccelN = 0;
  for (int i = 0; i < 3; i++) gSum[i] = gSumSq[i] = aSum[i] = 0.0;
}

// CALIBRATE [seconds]: redo the static gyro-bias calibration (table must stay still). Idle only.
// Best done after ~10 min powered: the BNO085 z bias drifts ~0.007 deg/s (~25 deg/h of yaw)
// over the first ~5 min while the chip warms up (static tests 2026-10-15/16).
static void startCalibration(const Command& c, CommandLink& from) {
  if (state != State::Idle) {
    from.send("# CALIBRATE REFUSED: not idle\n");
    return;
  }
  if (c.hasValue) {
    float s = c.value;
    if (s < 10.0f) s = 10.0f;
    if (s > 900.0f) s = 900.0f;
    calDurationMs = (uint32_t)(s * 1000.0f);
  }
  state = State::Calibrating;
  haveLastGyro = false;
  resetCalibration();
  char msg[72];
  snprintf(msg, sizeof(msg), "# CALIBRATING: hold the table still for %lu s\n",
           (unsigned long)(calDurationMs / 1000));
  sayAll(msg);
}

// Track the BNO085's gyro report sequence numbers and arrival gaps.
static void trackGyroSequence(const ImuSample& s) {
  if (haveGyroSeq) {
    const uint8_t dseq = (uint8_t)(s.seq - lastGyroSeq);  // wraps mod 256
    const float gapMs = (uint32_t)(s.t_us - lastGyroSeqUs) * 1.0e-3f;
    if (dseq > 1) seqMissed += dseq - 1;
    if (gapMs > params::kImuGapMs && dseq == 1) chipGaps++;
    if (gapMs > maxGyroGapMs) maxGyroGapMs = gapMs;
  }
  lastGyroSeq = s.seq;
  lastGyroSeqUs = s.t_us;
  haveGyroSeq = true;
}

// Wheel speed as the active loop sees it: low-passed in Manual, raw filter otherwise.
static float shownSpeed(int i) { return state == State::Manual ? manualSpeed[i] : wheels[i].speed(); }

static void printUsbTelemetry() {
  const Quat q = ekf.quaternion();
  const Vec3 e = ekf.eulerRad() * kRadToDeg;
  const Vec3 b = ekf.gyroBias() * kRadToDeg;
  const Vec3 s = ekf.sigmaRad() * kRadToDeg;
  Serial.printf(
      "%lu,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%.1f,%.1f,%.1f,%.1f,%c,"
      "%.3f,%.3f,%.3f,%.3f,%c,%.2f\n",
      millis(), q.w, q.x, q.y, q.z, e.x, e.y, e.z, b.x, b.y, b.z, s.x, s.y, s.z,
      static_cast<int>(lastAccelResult), shownSpeed(0), shownSpeed(1), shownSpeed(2), shownSpeed(3),
      stateChar(), wheels[0].axisDuty(), wheels[1].axisDuty(), wheels[2].axisDuty(),
      wheels[3].axisDuty(), magAid.modeChar(), magAid.headingResidualDeg());
}

// RAW row (one per magnetometer sample): R,t_us,mag x/y/z (uT),encoder counts A-D,
// wheel speed A-D (rad/s),duty A-D,gyro z (rad/s),EKF yaw (deg),state. Counts give each motor
// shaft's angle (64 counts per motor turn), which sets the static field of its magnets.
static void printRawRow(uint32_t tUs) {
  const Vec3& m = latestMag;
  Serial.printf("R,%lu,%.3f,%.3f,%.3f,%ld,%ld,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.5f,%.3f,%c,%c,%.2f\n",
                (unsigned long)tUs, m.x, m.y, m.z, wheels[0].count(), wheels[1].count(), wheels[2].count(),
                wheels[3].count(), wheels[0].speed(), wheels[1].speed(), wheels[2].speed(), wheels[3].speed(),
                wheels[0].axisDuty(), wheels[1].axisDuty(), wheels[2].axisDuty(), wheels[3].axisDuty(),
                latestGyro.z, ekf.eulerRad().z * kRadToDeg, stateChar(), magAid.modeChar(),
                magAid.headingResidualDeg());
}

// One log row per gyro sample, stamped with that sample's arrival time.
static void logData(uint32_t sampleUs) {
  if (!logClockStarted) {
    logClockUs = sampleUs;
    logClockStarted = true;
  } else {
    logClockUs += (uint32_t)(sampleUs - logLastRawUs);  // wrap-safe delta
  }
  logLastRawUs = sampleUs;

  LogRow r;
  r.ms = (double)logClockUs * 1.0e-3;
  r.gyro = latestGyro;
  r.accel = latestAccel;
  r.mag = latestMag;
  constexpr float kRadSToRpm = 60.0f / 6.2831853f;
  for (int i = 0; i < kN; i++) {
    r.rpm[i] = wheels[i].speed() * kRadSToRpm;
    r.duty[i] = wheels[i].axisDuty();
  }
  r.q = ekf.quaternion();
  r.eulerDeg = ekf.eulerRad() * kRadToDeg;
  r.bias = ekf.gyroBias();
  r.sigmaDeg = ekf.sigmaRad() * kRadToDeg;
  r.accelStatus = static_cast<int>(lastAccelResult);
  r.state = stateChar();
  r.gyroSeq = lastGyroSeq;
  r.seqMissed = seqMissed;
  r.testStep = magStep;
  r.magMode = magAid.modeChar();
  r.magResidDeg = magAid.headingResidualDeg();
  if (demo.active()) {
    r.demoPhase = demo.phaseChar();
    r.rateRef = demo.rateRef();
    r.yawRefDeg = demo.yawRef() * kRadToDeg;
  }
  wheelStatus(r.wheelStatus);
  if (state == State::Armed) r.errDeg = ctrl.angleError() * kRadToDeg;
  r.faultPhase = fault.phaseChar();
  if (slew.active()) {  // SLEW TEST: phase S/W/H, |reference rate|, reference yaw, target index
    r.demoPhase = slew.phaseChar();
    r.rateRef = slew.refRate();
    r.yawRefDeg = slew.refYawDeg();
    r.testStep = slew.index();
  }
  logger.logRow(r);
}

// Manual (SPEED) mode: closed-loop speed setpoint for one wheel or all (Command::kAllWheels),
// entering Manual from Idle. Setpoints ramp at kManualAccelRadS2; overspeed disarms.
// ---- MAG MAP: step the wheels through mag_map::kSteps (Manual mode) while logging ----
static void applyMagStep() {
  constexpr float kRpmToRadS = 6.2831853f / 60.0f;
  const mag_map::Step& s = mag_map::kSteps[magStep];
  for (int i = 0; i < kN; i++) manualTarget[i] = s.rpm[i] * kRpmToRadS;
  magStepStartMs = millis();
  char msg[80];
  snprintf(msg, sizeof(msg), "# MAGMAP step %d/%d: A=%d B=%d C=%d D=%d rpm\n", magStep + 1, mag_map::kNumSteps,
           s.rpm[0], s.rpm[1], s.rpm[2], s.rpm[3]);
  sayAll(msg);
}

static void startMagMap(CommandLink& from) {
  if (state != State::Idle) {
    from.send("# MAGMAP REFUSED: not idle (wait for calibration / DISARM first)\n");
    return;
  }
  for (int i = 0; i < kN; i++) {
    if (fabsf(wheels[i].speed()) > params::kDemoStartMaxWheelRadS) {
      from.send("# MAGMAP REFUSED: wheels still turning\n");
      return;
    }
  }
  if (!logger.sdActive()) logger.startFile("MAGMAP");
  testWheel = -1;
  for (int i = 0; i < kN; i++) {
    speedLoops[i].reset();
    wheelCmd[i] = manualTarget[i] = 0.0f;
  }
  state = State::Manual;
  char msg[112];
  snprintf(msg, sizeof(msg), "# MAGMAP START: %d steps x %lu s (~%lu min), table must be FIXED%s%s\n",
           mag_map::kNumSteps, (unsigned long)(params::kMagMapStepMs / 1000),
           (unsigned long)(mag_map::kNumSteps * params::kMagMapStepMs / 60000 + 1),
           logger.sdActive() ? ", log " : ", NO SD LOG", logger.sdActive() ? logger.fileName() : "");
  sayAll(msg);
  magStep = 0;
  applyMagStep();
}

// Called every loop(): advance the schedule; end with the wheels stopped (logging continues).
static void updateMagMap() {
  if (magStep < 0) return;
  if (state != State::Manual) {  // overspeed or another command left Manual mode
    magStep = -1;
    sayAll("# MAGMAP ABORTED: left Manual mode\n");
    return;
  }
  if (millis() - magStepStartMs < params::kMagMapStepMs) return;
  if (++magStep >= mag_map::kNumSteps) {
    disarm("MAGMAP_COMPLETE");  // wheels stop; SD logging continues until power off
    return;
  }
  applyMagStep();
}

static bool setManualSpeed(int wheel, float rpm, CommandLink& from) {
  if (state != State::Idle && state != State::Manual) {
    from.send("# SPEED REFUSED: not idle/manual\n");
    return false;
  }
  if (state == State::Idle) {
    testWheel = -1;
    for (int i = 0; i < kN; i++) {
      speedLoops[i].reset();
      wheelCmd[i] = manualTarget[i] = 0.0f;
    }
    state = State::Manual;
  }
  constexpr float kRpmToRadS = 6.2831853f / 60.0f;
  float w = rpm * kRpmToRadS;
  if (w > params::kWheelMaxSpeedRadS) w = params::kWheelMaxSpeedRadS;
  if (w < -params::kWheelMaxSpeedRadS) w = -params::kWheelMaxSpeedRadS;
  for (int i = 0; i < kN; i++)
    if (wheel == Command::kAllWheels || wheel == i) manualTarget[i] = w;
  char msg[72];
  snprintf(msg, sizeof(msg), "# MANUAL rpm A=%.0f B=%.0f C=%.0f D=%.0f\n", manualTarget[0] / kRpmToRadS,
           manualTarget[1] / kRpmToRadS, manualTarget[2] / kRpmToRadS, manualTarget[3] / kRpmToRadS);
  sayAll(msg);
  return true;
}

static void handleCommand(const Command& c, CommandLink& from) {
  switch (c.cmd) {
    case Cmd::Kill:
    case Cmd::Disarm:
      disarm(c.cmd == Cmd::Kill ? "KILL" : "DISARM");
      break;
    case Cmd::Arm:
      arm(from);
      break;
    case Cmd::Ping:
      from.send("PONG\n");
      break;
    case Cmd::Status: {
      const Vec3 e = ekf.eulerRad() * kRadToDeg;
      const Vec3 s = ekf.sigmaRad() * kRadToDeg;
      char ws[kN + 1];
      wheelStatus(ws);
      char msg[320];
      snprintf(msg, sizeof(msg),
               "S,%c,%.1f,%.1f,%.1f,sig_yaw=%.2f,%.0f,%.0f,%.0f,%.0f,sd=%s,drop=%lu,demo=%c,"
               "miss=%lu,chipgap=%lu,maxgap=%.0f,lat=%.2f,int=%lu,svc=%lu/%lu,ev=%lu,rst=%lu,en=%lu/%d,intpin=%d,"
               "wheels=%c%c%c%c,mag=%c,mres=%.2f,bz=%.5f,wst=%s,fault=%c\n",
               stateChar(), e.x, e.y, e.z, s.z, wheels[0].speed(), wheels[1].speed(), wheels[2].speed(),
               wheels[3].speed(), logger.sdActive() ? logger.fileName() : "off",
               (unsigned long)logger.droppedRows(), slew.active() ? slew.phaseChar() : demo.phaseChar(), (unsigned long)seqMissed,
               (unsigned long)chipGaps, maxGyroGapMs, imu.maxLatencyUs() * 1.0e-3f,
               (unsigned long)imu.intCount(), (unsigned long)imu.serviceCount(),
               (unsigned long)imu.freshCount(),
               (unsigned long)imu.eventCount(), (unsigned long)imu.resetCount(),
               (unsigned long)imu.enableCount(), imu.lastEnableOk() ? 1 : 0,
               kPinImuInt >= 0 ? digitalRead(kPinImuInt) : -1, wheelEnabled[0] ? 'A' : '-',
               wheelEnabled[1] ? 'B' : '-', wheelEnabled[2] ? 'C' : '-', wheelEnabled[3] ? 'D' : '-', magAid.modeChar(),
               magAid.headingResidualDeg(), ekf.gyroBias().z * kRadToDeg, ws, fault.phaseChar());
      from.send(msg);
      break;
    }
    case Cmd::DemoSpin:
      startDemo(from, false);
      break;
    case Cmd::DemoLevel:
      startDemo(from, true);
      break;
    case Cmd::SlewTest:
      startSlew(from);
      break;
    case Cmd::SlewList:
      for (int i = 0; i < params::kSlewNumTargets; i++) sayTarget("SLEW target", i);
      break;
    case Cmd::SlewSet:
      if (state != State::Idle) {
        from.send("# SLEW SET REFUSED: not idle\n");
      } else if (!slew.setTarget(c.wheel, c.v[0], c.v[1], c.v[2])) {
        from.send("# SLEW SET REFUSED: target 1-4, tilt <= 25 deg\n");
      } else {
        sayTarget("SLEW target set", c.wheel);
      }
      break;
    case Cmd::Calibrate:
      startCalibration(c, from);
      break;
    case Cmd::FaultTest:
      startFaultTest(c, from);
      break;
    case Cmd::FaultClear:
      clearFaults(from);
      break;
    case Cmd::Stop:
      if (slew.active()) {
        if (slew.done()) {
          char msg[64];
          snprintf(msg, sizeof(msg), "# SLEW TEST STOPPED: %d/%d PASS\n", slew.passCount(), params::kSlewNumTargets);
          sayAll(msg);
        }
        disarm(slew.done() ? "SLEW_TEST_DONE" : "SLEW_STOPPED");
      } else if (magStep >= 0) {
        disarm("MAGMAP_STOPPED");
      } else {
        stopDemo(from);
      }
      break;
    case Cmd::MagMap:
      startMagMap(from);
      break;
    case Cmd::Wheel: {
      // WHEEL <A-D> ON|OFF: take a wheel out of (or back into) service. Idle only.
      if (state != State::Idle) {
        from.send("# WHEEL REFUSED: not idle\n");
        break;
      }
      wheelEnabled[c.wheel] = c.value != 0.0f;
      wheelDead[c.wheel] = false;
      deadDet[c.wheel].reset();
      encFaultS[c.wheel] = 0.0f;
      prevSpeed[c.wheel] = wheels[c.wheel].speed();
      char msg[48];
      snprintf(msg, sizeof(msg), "# WHEEL %c %s\n", 'A' + c.wheel, wheelEnabled[c.wheel] ? "ON" : "OFF");
      sayAll(msg);
      break;
    }
    case Cmd::MagAidOn:
    case Cmd::MagAidOff:
      magAid.setEnabled(c.cmd == Cmd::MagAidOn);
      sayAll(magAid.enabled() ? "# MAG AID ON (relearns its references)\n" : "# MAG AID OFF (gyro-only yaw)\n");
      break;
    case Cmd::Kick: {
      // KICK <deg>: test only - rotate the yaw estimate (not the table) to watch the aid pull it back.
      ekf.perturbYaw(c.value * kDegToRad);
      char msg[48];
      snprintf(msg, sizeof(msg), "# KICK yaw estimate %+.1f deg\n", c.value);
      sayAll(msg);
      break;
    }
    case Cmd::RawOn:
    case Cmd::RawOff:
      usbRaw = c.cmd == Cmd::RawOn;
      from.send(usbRaw ? "# RAW ON\n" : "# RAW OFF\n");
      break;
    case Cmd::BeginStatic: {
      // "BEGIN STATIC TEST [rpm]": SD log + all four wheels held at a constant speed (default
      // kStaticTestWheelRpm; 0 = motors off) to measure magnetometer/IMU behaviour with the
      // motors running. DISARM/KILL stops the wheels; logging continues until power off.
      if (state == State::Calibrating) {
        from.send("# BEGIN REFUSED: still calibrating - keep the table still and retry\n");
        break;
      }
      const float rpm = c.hasValue ? c.value : params::kStaticTestWheelRpm;
      if (rpm != 0.0f && state != State::Idle && state != State::Manual) {
        from.send("# BEGIN REFUSED: not idle (DISARM first)\n");
        break;
      }
      char msg[112];
      if (logger.sdActive()) {
        snprintf(msg, sizeof(msg), "# STATIC TEST already running: %s\n", logger.fileName());
        from.send(msg);
        break;
      }
      if (!logger.startFile("STATIC")) break;  // startFile reports the reason itself
      snprintf(msg, sizeof(msg), "# STATIC TEST STARTED: logging 100 Hz to %s until power off, wheels %.0f rpm\n",
               logger.fileName(), rpm);
      sayAll(msg);
      if (rpm != 0.0f) setManualSpeed(Command::kAllWheels, rpm, from);
      break;
    }
    case Cmd::Test: {
      if (state != State::Idle) {
        from.send("# TEST REFUSED: not idle\n");
        break;
      }
      float d = c.value;
      if (d > params::kTestMaxDuty) d = params::kTestMaxDuty;
      if (d < -params::kTestMaxDuty) d = -params::kTestMaxDuty;
      testWheel = c.wheel;
      testDuty = d;
      testEndMs = millis() + params::kTestDurationMs;
      break;
    }
    case Cmd::Speed:
      setManualSpeed(c.wheel, c.value, from);
      break;
    case Cmd::Counts: {
      long n[kN];
      for (int i = 0; i < kN; i++) n[i] = wheels[i].count();
      const float k = 1.0f / params::kEncoderCountsPerRev;
      char msg[128];
      snprintf(msg, sizeof(msg), "COUNTS A=%ld B=%ld C=%ld D=%ld turns A=%.3f B=%.3f C=%.3f D=%.3f\n", n[0],
               n[1], n[2], n[3], n[0] * k, n[1] * k, n[2] * k, n[3] * k);
      from.send(msg);
      break;
    }
    case Cmd::Zero:
      for (int i = 0; i < kN; i++) wheels[i].zeroCount();
      from.send("# ZERO counts\n");
      break;
    default:
      from.send("# ?\n");
      break;
  }
}

static void pollCommands() {
  Command c;
  while (xbee.poll(c)) handleCommand(c, xbee);
  while (usb.poll(c)) handleCommand(c, usb);
}

// Returns a reason string if the Armed state must be abandoned, else nullptr.
static const char* safetyViolation() {
  const Vec3 e = ekf.eulerRad();
  // SLEW TEST keeps the table under 25 deg (the bubble has no top).
  const float lim = (slew.active() ? params::kSlewMaxTiltDeg : params::kMaxTiltDeg) * kDegToRad;
  if (fabsf(e.x) > lim || fabsf(e.y) > lim) return "TILT";
  if (norm(ekf.bodyRate()) > params::kMaxRateRadS) return "RATE";
  for (int i = 0; i < kN; i++)
    if (wheelEnabled[i] && fabsf(wheels[i].speed()) > params::kWheelOverspeedRadS) return "WHEEL_OVERSPEED";
  return nullptr;
}

// Attitude loop: runs on every gyro sample while armed.
static void controlStep(float dt) {
  const char* bad = safetyViolation();
  if (bad) {
    disarm(bad);
    return;
  }

  if (slew.active() && !stepSlew(dt)) return;

  if (demo.active()) {
    const Vec3 e = ekf.eulerRad();
    switch (demo.step(dt, e.x, e.y)) {
      case DemoSpin::Event::Leveled: {
        char msg[112];
        if (demo.levelOnly()) {
          snprintf(msg, sizeof(msg), "# DEMO LEVEL: settled after %.1f s at rest tilt roll %.2f pitch %.2f deg\n",
                   demo.elapsed(), demo.restRoll() * kRadToDeg, demo.restPitch() * kRadToDeg);
        } else {
          snprintf(msg, sizeof(msg), "# DEMO LEVEL: spinning up to %.1f rpm\n",
                   demo.targetRate() * 60.0f / 6.2831853f);
        }
        sayAll(msg);
        break;
      }
      case DemoSpin::Event::LevelTimeout:
        disarm("DEMO_LEVEL_TIMEOUT");
        return;
      case DemoSpin::Event::HoldStarted:
        sayAll("# DEMO HOLD\n");
        break;
      case DemoSpin::Event::SpinDownStarted:
        sayAll("# DEMO SPIN DOWN\n");
        break;
      case DemoSpin::Event::Settling:
        sayAll("# DEMO SETTLE\n");
        break;
      case DemoSpin::Event::Complete:
        disarm("DEMO_COMPLETE");
        return;
      default:
        break;
    }
    // Momentum budget guard: a wheel near its cap means the inertia model is off (or drag).
    float wmax = 0.0f;
    for (int i = 0; i < kN; i++) wmax = fmaxf(wmax, fabsf(wheels[i].speed()));
    if (wmax > params::kDemoWheelAbortRadS) {
      if (demo.phase() == DemoPhase::Level || demo.phase() == DemoPhase::LevelHold) {
        if (demo.levelOnly()) reportLevelHold("DEMO LEVEL WHEEL LIMIT");
        disarm("DEMO_WHEEL_LIMIT");
        return;
      }
      if (demo.requestStop() == DemoSpin::Event::SpinDownStarted)
        sayAll("# DEMO ABORT: wheel near limit, spinning down\n");
    }
    ctrl.setTrajectory(demo.ref().q, demo.ref().w, demo.ref().a);
  }

  const Vec3 omega = ekf.bodyRate();
  const Vec3 tau = ctrl.update(ekf.quaternion(), omega, dt, wheelMomentum(omega));
  if (fault.active())
    reportFault(fault.step(dt, ctrl.angleError() * kRadToDeg, fault.wheel() >= 0 ? wheels[fault.wheel()].speed() : 0.0f));

  float u[kN];
  if (!allocate(tau, wheelEnabled, u)) {
    disarm("ALLOC");
    return;
  }
  bool all = true;
  float spd[kN];
  for (int i = 0; i < kN; i++) {
    all = all && wheelEnabled[i];
    spd[i] = wheels[i].speed();
  }
  // Null mode: hold the [+1,-1,+1,-1] wheel-speed combination at the bias (no body torque).
  // DEMO SPIN keeps the wheels at ~100 rpm so no shaft sits still (magnetometer, mag_map.h).
  float nullBias = demo.active() ? demo.wheelBias() : params::kNullBiasRadS;
  // FAULT TEST keeps the wheels turning so a dead motor shows up at once (wheel_fault.h).
  if (fault.wheelBias() > nullBias) nullBias = fault.wheelBias();
  if (all) addNullSpaceTorque(spd, params::kWheelInertia, params::kNullGain, nullBias, u);

  for (int i = 0; i < kN; i++) {
    if (!wheelEnabled[i]) continue;
    float cmd = wheelCmd[i] + (u[i] / params::kWheelInertia) * dt;
    // Keep the setpoint within what the motor can follow (limits wind-up).
    const float lead = params::kWheelCmdLeadRadS;
    if (cmd > spd[i] + lead) cmd = spd[i] + lead;
    if (cmd < spd[i] - lead) cmd = spd[i] - lead;
    if (cmd > params::kWheelMaxSpeedRadS) cmd = params::kWheelMaxSpeedRadS;
    if (cmd < -params::kWheelMaxSpeedRadS) cmd = -params::kWheelMaxSpeedRadS;
    wheelCmd[i] = cmd;
  }
}

// Inner loop at kSpeedLoopHz: encoder speeds, speed PI, motor outputs, bench test.
static void wheelLoop(float dt) {
  // Manual-mode low-pass runs in every state so it is already settled when SPEED starts.
  const float lpfA = dt / (params::kManualSpeedLpfTauS + dt);
  for (int i = 0; i < kN; i++) {
    wheels[i].updateSpeed(dt);
    manualSpeed[i] += lpfA * (wheels[i].speed() - manualSpeed[i]);
  }
  for (int i = 0; i < kN; i++) {
    const float w = wheels[i].speed();
    const bool jump = dt > 0.0f && fabsf(w - prevSpeed[i]) > params::kEncFaultMaxAccelRadS2 * dt;
    prevSpeed[i] = w;
    const bool stuck = fabsf(wheels[i].axisDuty()) > params::kEncFaultDuty && fabsf(w) < params::kEncFaultSpeedRadS;
    encFaultS[i] = stuck ? encFaultS[i] + dt : 0.0f;
    if (wheelEnabled[i] && (jump || encFaultS[i] > params::kEncFaultTimeS)) {
      wheelEnabled[i] = false;
      wheels[i].stop();
      char msg[96];
      snprintf(msg, sizeof(msg), "# WHEEL FAULT %c: %s - wheel disabled (WHEEL %c ON to re-enable)\n", 'A' + i,
               jump ? "speed jump (encoder glitch)" : "driven but encoder shows no motion", 'A' + i);
      sayAll(msg);
      onWheelFailed(i);
    }
    // Dead motor: driven hard (Armed) but the wheel doesn't accelerate. Uses the duty applied over
    // the last interval, which is what moved the wheel to its new speed.
    if (state != State::Armed || !wheelEnabled[i]) {
      deadDet[i].reset();
    } else if (deadDet[i].update(wheels[i].axisDuty() - w / params::kWheelFreeSpeedRadS, w, dt)) {
      wheelEnabled[i] = false;
      wheelDead[i] = true;
      wheels[i].coast();
      char msg[128];
      snprintf(msg, sizeof(msg),
               "# WHEEL FAULT %c: dead motor (driven, not accelerating) - disabled, coasting; control on the rest\n",
               'A' + i);
      sayAll(msg);
      onWheelFailed(i);
    }
  }

  const bool testing = testWheel >= 0 && (int32_t)(millis() - testEndMs) < 0;
  if (testWheel >= 0 && !testing) {
    char msg[64];
    snprintf(msg, sizeof(msg), "# TEST %c duty=%.2f speed=%.1f rad/s\n", 'A' + testWheel, testDuty,
             wheels[testWheel].speed());
    sayAll(msg);
    testWheel = -1;
  }

  if (state == State::Manual) {
    for (int i = 0; i < kN; i++) {
      if (wheelEnabled[i] && fabsf(wheels[i].speed()) > params::kWheelOverspeedRadS) {
        disarm("WHEEL_OVERSPEED");
        return;
      }
    }
  }

  for (int i = 0; i < kN; i++) {
    if (!wheelEnabled[i]) {
      // Disabled: never driven, in any mode. A dead motor coasts (zero current); others brake.
      if (wheelDead[i]) {
        wheels[i].coast();
      } else {
        wheels[i].stop();
      }
    } else if (state == State::Manual) {
      // Ramp the setpoint toward the SPEED target (limits overshoot and current surges).
      // The integrator is held while ramping so it doesn't wind up and overshoot at the end.
      const float step = params::kManualAccelRadS2 * dt;
      const float diff = manualTarget[i] - wheelCmd[i];
      const bool ramping = fabsf(diff) > step;
      wheelCmd[i] += ramping ? (diff > 0.0f ? step : -step) : diff;
      if (manualTarget[i] == 0.0f && wheelCmd[i] == 0.0f && fabsf(manualSpeed[i]) < params::kManualReleaseRadS) {
        // Stopped at a 0 target: release the drive. The integrator otherwise held ~-0.06 duty in a
        // stopped motor (2026-10-27), and that current moved the magnetometer by microtesla.
        speedLoops[i].reset();
        wheels[i].stop();
        continue;
      }
      wheels[i].setDuty(speedLoops[i].update(wheelCmd[i], manualSpeed[i], dt,
                                             params::kManualFreeSpeedRadS, !ramping),
                        dt);
    } else if (state == State::Armed && wheelEnabled[i]) {
      wheels[i].setDuty(speedLoops[i].update(wheelCmd[i], wheels[i].speed(), dt), dt);
    } else if (testing && i == testWheel) {
      wheels[i].setDuty(testDuty, dt);
    } else {
      wheels[i].stop();
    }
  }
}

static void handleCalibrationSample(const ImuSample& s) {
  if (s.kind == ImuSample::Gyro) {
    calGyroN++;
    gSum[0] += s.v.x;  gSumSq[0] += (double)s.v.x * s.v.x;
    gSum[1] += s.v.y;  gSumSq[1] += (double)s.v.y * s.v.y;
    gSum[2] += s.v.z;  gSumSq[2] += (double)s.v.z * s.v.z;
  } else if (s.kind == ImuSample::Accel) {
    calAccelN++;
    aSum[0] += s.v.x;
    aSum[1] += s.v.y;
    aSum[2] += s.v.z;
  }

  const uint32_t elapsed = millis() - calStartMs;
  if (elapsed < calDurationMs) {
    if (millis() - calProgressMs >= params::kCalProgressMs) {
      calProgressMs = millis();
      char msg[56];
      snprintf(msg, sizeof(msg), "# calibrating: %lu / %lu s, keep still\n", (unsigned long)(elapsed / 1000),
               (unsigned long)(calDurationMs / 1000));
      sayAll(msg);
    }
    return;
  }

  if (calGyroN < kMinCalSamples || calAccelN < kMinCalSamples / 2) {
    sayAll("# calibration: too few IMU samples, retrying\n");
    resetCalibration();
    return;
  }

  Vec3 mean, stdv;
  float* m[3] = {&mean.x, &mean.y, &mean.z};
  float* sd[3] = {&stdv.x, &stdv.y, &stdv.z};
  for (int i = 0; i < 3; i++) {
    const double mu = gSum[i] / calGyroN;
    const double var = gSumSq[i] / calGyroN - mu * mu;
    *m[i] = (float)mu;
    *sd[i] = sqrtf(var > 0.0 ? (float)var : 0.0f);
  }

  const bool still = stdv.x < kCalMaxGyroStd && stdv.y < kCalMaxGyroStd &&
                     stdv.z < kCalMaxGyroStd && norm(mean) < kCalMaxGyroMean;
  if (!still) {
    sayAll("# calibration: table is moving, HOLD STILL - restarting\n");
    resetCalibration();
    return;
  }

  const Vec3 accelMean = {(float)(aSum[0] / calAccelN), (float)(aSum[1] / calAccelN),
                          (float)(aSum[2] / calAccelN)};
  float worst = stdv.x;
  if (stdv.y > worst) worst = stdv.y;
  if (stdv.z > worst) worst = stdv.z;
  const float biasSigma = worst / sqrtf((float)calGyroN);  // std error of the mean

  if (!ekf.initialize(accelMean, mean, biasSigma)) {
    sayAll("# calibration: bad gravity vector, retrying\n");
    resetCalibration();
    return;
  }

  char msg[96];
  snprintf(msg, sizeof(msg), "# calibrated: bias[deg/s]=%.4f,%.4f,%.4f (N=%lu, %lu s)\n", mean.x * kRadToDeg,
           mean.y * kRadToDeg, mean.z * kRadToDeg, (unsigned long)calGyroN,
           (unsigned long)(calDurationMs / 1000));
  sayAll(msg);
  Serial.println(
      "# t_ms,qw,qx,qy,qz,roll,pitch,yaw,bx,by,bz,sig_roll,sig_pitch,sig_yaw,accel_status,w0,w1,w2,w3,state,"
      "dutyA,dutyB,dutyC,dutyD,mag_mode,mag_resid_deg");
  haveLastGyro = false;
  magAid.reset();  // the stopped reference is learned now, at yaw 0
  state = State::Idle;
  sayAll("# READY (idle). Send ARM to hold level.\n");
}

void setup() {
  Serial.begin(115200);
  XBEE_SERIAL.begin(params::kXbeeBaud);
  XBEE_SERIAL.addMemoryForWrite(xbeeTxExtra, sizeof(xbeeTxExtra));
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  analogWriteResolution(params::kPwmBits);
  for (int i = 0; i < kN; i++) wheels[i].begin();

  logger.begin();

  IMU_I2C_BUS.begin();
  IMU_I2C_BUS.setClock(400000);
  // Keep retrying (each attempt hardware-resets the chip) and keep saying why, so the message
  // is seen even if the USB/XBee monitor was opened late. Halts in setup only.
  while (!imu.begin(IMU_I2C_BUS, kImuI2cAddr)) {
    stopAllWheels();
    char msg[112];
    // Wiring diagnostics: idle level of RST and INT with neither driven (RST has a pull-up on
    // the breakout, INT is driven by the chip), and every address that ACKs on the bus.
    if (kPinImuReset >= 0) pinMode(kPinImuReset, INPUT);
    if (kPinImuInt >= 0) pinMode(kPinImuInt, INPUT);
    delay(5);
    char found[48] = "";
    for (uint8_t a = 0x08; a < 0x78; a++) {
      IMU_I2C_BUS.beginTransmission(a);
      if (IMU_I2C_BUS.endTransmission() == 0 && strlen(found) < sizeof(found) - 6) {
        char h[6];
        snprintf(h, sizeof(h), "0x%02X ", a);
        strcat(found, h);
      }
    }
    snprintf(msg, sizeof(msg), "# ERROR: BNO085 not responding. RST(%d)=%d INT(%d)=%d i2c: %s\n",
             kPinImuReset, kPinImuReset >= 0 ? digitalRead(kPinImuReset) : -1, kPinImuInt,
             kPinImuInt >= 0 ? digitalRead(kPinImuInt) : -1, found[0] ? found : "none");
    sayAll(msg);
    delay(1000);
  }
  {
    char msg[72];
    snprintf(msg, sizeof(msg), "# BNO085 ready. HOLD THE TABLE STILL for %lu s calibration.\n",
             (unsigned long)(calDurationMs / 1000));
    sayAll(msg);
  }
  resetCalibration();
  lastGyroMs = millis();
  lastWheelUs = micros();
}

// Replaces the Teensy core's weak yield(). SdFat calls yield() while it waits on the SD card
// (and delay() calls it too), so reading the IMU here keeps it serviced through SD write stalls;
// the samples are queued and processed by loop() with their own timestamps. The core's default
// yield() only dispatches serialEvent*() (not used here) and EventResponder, kept below.
void yield() {
  imu.background();
  if (yield_active_check_flags & YIELD_CHECK_EVENT_RESPONDER) EventResponder::runFromYield();
}

void loop() {
  pollCommands();  // KILL is handled here, before any IMU work

  // Drain everything the IMU has queued, in arrival order.
  ImuSample s;
  while (imu.poll(s)) {
    if (s.kind == ImuSample::Gyro) {
      latestGyro = s.v;
      trackGyroSequence(s);
    } else if (s.kind == ImuSample::Accel) latestAccel = s.v;
    else if (s.kind == ImuSample::Mag) {
      latestMag = s.v;
      const float dtMag = haveLastMag ? (uint32_t)(s.t_us - lastMagUs) * 1.0e-6f : 0.0f;
      lastMagUs = s.t_us;
      haveLastMag = true;
      if (state != State::Calibrating && ekf.initialized() && dtMag > 0.0f && dtMag < 0.1f) {
        long cnt[kN];
        float spd[kN];
        for (int i = 0; i < kN; i++) {
          cnt[i] = wheels[i].count();
          spd[i] = wheels[i].speed();
        }
        Vec3 mUse, ref;
        lastMagResult = MagResult::Gated;
        // A dead-motor wheel still moves (coasting) and its encoder works: keep it in the mag aid's
        // wheel-state check so its new stopping place can't leave a stale field offset.
        bool magWheels[kN];
        for (int i = 0; i < kN; i++) magWheels[i] = wheelEnabled[i] || wheelDead[i];
        if (magAid.update(s.v, cnt, spd, magWheels, ekf.quaternion(), dtMag, mUse, ref))
          lastMagResult = ekf.updateMagYaw(mUse, ref, params::kMagHeadingSigmaRad);
      }
      if (usbRaw) printRawRow(s.t_us);
    }

    if (state == State::Calibrating) {
      handleCalibrationSample(s);
      if (s.kind == ImuSample::Gyro) {
        lastGyroMs = millis();
        logData(s.t_us);
      }
      continue;
    }

    if (s.kind == ImuSample::Gyro) {
      lastGyroMs = millis();
      if (haveLastGyro) {
        const float dt = (uint32_t)(s.t_us - lastGyroUs) * 1.0e-6f;
        // Integrate gaps up to the IMU timeout with the new sample's rate (the BNO085 stalls for
        // 60-90 ms every ~22 s; skipping those would lose the rotation during them). Longer gaps
        // are dropped (and disarm when Armed).
        if (dt > 0.0f && dt <= params::kImuTimeoutMs * 1.0e-3f) {
          ekf.predict(s.v, dt);
          if (state == State::Armed) controlStep(dt);
        }
      }
      lastGyroUs = s.t_us;
      haveLastGyro = true;
      logData(s.t_us);
    } else if (s.kind == ImuSample::Accel) {
      lastAccelResult = ekf.updateAccel(s.v);
    }
  }

  if (state == State::Armed && millis() - lastGyroMs > params::kImuTimeoutMs) disarm("IMU_TIMEOUT");

  const uint32_t nowUs = micros();
  const uint32_t periodUs = (uint32_t)(1.0e6f / params::kSpeedLoopHz);
  if ((uint32_t)(nowUs - lastWheelUs) >= periodUs) {
    wheelLoop((uint32_t)(nowUs - lastWheelUs) * 1.0e-6f);
    lastWheelUs = nowUs;
  }

  updateMagMap();
  logger.service();  // rows are produced per gyro sample above

  if (!usbRaw && state != State::Calibrating && millis() - lastUsbMs >= params::kUsbTelemetryMs) {
    lastUsbMs = millis();
    printUsbTelemetry();
  }
}
