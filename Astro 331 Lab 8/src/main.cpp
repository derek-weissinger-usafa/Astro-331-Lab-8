#include <Arduino.h>

#include "attitude_ctrl.h"
#include "attitude_ekf.h"
#include "command_link.h"
#include "control_params.h"
#include "data_logger.h"
#include "frames.h"
#include "imu_bno085.h"
#include "pins.h"
#include "wheel_geometry.h"
#include "wheel_motor.h"
#include "wheel_speed_ctrl.h"

// Reaction-wheel attitude hold (level, yaw 0) on a BNO085 gyro+accel MEKF.
// States: Calibrating (hold still, ~3 s) -> Idle (wheels off) -> Armed (hold).
// Commands arrive over the XBee (Serial8) and, for bench use, USB Serial.
// Sensors and control run at 100 Hz; raw data is logged at 20 Hz to the SD card
// and every 4th row of that goes out over the XBee (data_logger.h).

using namespace wheel_geometry;

static Bno085Imu imu(kPinImuReset);
static AttitudeEKF ekf;
static AttitudeController ctrl;
static CommandLink xbee(XBEE_SERIAL);
static CommandLink usb(Serial);
static DataLogger logger(XBEE_SERIAL);

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
static float wheelCmd[kN] = {0, 0, 0, 0};                 // wheel speed setpoints, rad/s
static float manualTarget[kN] = {0, 0, 0, 0};             // SPEED targets; wheelCmd ramps to these

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
static constexpr uint32_t kCalDurationMs = 3000;
static constexpr float kCalMaxGyroStd = 0.01f;    // rad/s per axis, else "not still"
static constexpr float kCalMaxGyroMean = 0.05f;   // rad/s, rejects a rotating table
static constexpr uint32_t kMinCalSamples = 100;

static uint32_t calStartMs = 0;
static uint32_t calGyroN = 0, calAccelN = 0;
static double gSum[3], gSumSq[3], aSum[3];

static uint32_t lastGyroUs = 0, lastGyroMs = 0;
static bool haveLastGyro = false;
static uint32_t lastUsbMs = 0, lastWheelUs = 0, lastLogUs = 0;
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

static void disarm(const char* reason) {
  stopAllWheels();
  testWheel = -1;
  if (state == State::Armed || state == State::Manual) state = State::Idle;
  char msg[48];
  snprintf(msg, sizeof(msg), "# DISARM %s\n", reason);
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

static void resetCalibration() {
  calStartMs = millis();
  calGyroN = calAccelN = 0;
  for (int i = 0; i < 3; i++) gSum[i] = gSumSq[i] = aSum[i] = 0.0;
}

static void printUsbTelemetry() {
  const Quat q = ekf.quaternion();
  const Vec3 e = ekf.eulerRad() * kRadToDeg;
  const Vec3 b = ekf.gyroBias() * kRadToDeg;
  const Vec3 s = ekf.sigmaRad() * kRadToDeg;
  Serial.printf(
      "%lu,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%.1f,%.1f,%.1f,%.1f,%c\n",
      millis(), q.w, q.x, q.y, q.z, e.x, e.y, e.z, b.x, b.y, b.z, s.x, s.y, s.z,
      static_cast<int>(lastAccelResult), wheels[0].speed(), wheels[1].speed(), wheels[2].speed(),
      wheels[3].speed(), stateChar());
}

static void logData() {
  LogRow r;
  r.ms = millis();
  r.gyro = latestGyro;
  r.accel = latestAccel;
  r.mag = latestMag;
  constexpr float kRadSToRpm = 60.0f / 6.2831853f;
  for (int i = 0; i < kN; i++) r.rpm[i] = wheels[i].speed() * kRadSToRpm;
  logger.logRow(r);
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
      char msg[96];
      snprintf(msg, sizeof(msg), "S,%c,%.1f,%.1f,%.1f,sig_yaw=%.2f,%.0f,%.0f,%.0f,%.0f\n", stateChar(),
               e.x, e.y, e.z, s.z, wheels[0].speed(), wheels[1].speed(), wheels[2].speed(),
               wheels[3].speed());
      from.send(msg);
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
    case Cmd::Speed: {
      if (state != State::Idle && state != State::Manual) {
        from.send("# SPEED REFUSED: not idle/manual\n");
        break;
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
      float w = c.value * kRpmToRadS;
      if (w > params::kWheelMaxSpeedRadS) w = params::kWheelMaxSpeedRadS;
      if (w < -params::kWheelMaxSpeedRadS) w = -params::kWheelMaxSpeedRadS;
      for (int i = 0; i < kN; i++)
        if (c.wheel == Command::kAllWheels || c.wheel == i) manualTarget[i] = w;
      char msg[72];
      snprintf(msg, sizeof(msg), "# MANUAL rpm A=%.0f B=%.0f C=%.0f D=%.0f\n", manualTarget[0] / kRpmToRadS,
               manualTarget[1] / kRpmToRadS, manualTarget[2] / kRpmToRadS, manualTarget[3] / kRpmToRadS);
      sayAll(msg);
      break;
    }
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
  const float lim = params::kMaxTiltDeg * kDegToRad;
  if (fabsf(e.x) > lim || fabsf(e.y) > lim) return "TILT";
  if (norm(ekf.bodyRate()) > params::kMaxRateRadS) return "RATE";
  for (int i = 0; i < kN; i++)
    if (fabsf(wheels[i].speed()) > params::kWheelOverspeedRadS) return "WHEEL_OVERSPEED";
  return nullptr;
}

// Attitude loop: runs on every gyro sample while armed.
static void controlStep(float dt) {
  const char* bad = safetyViolation();
  if (bad) {
    disarm(bad);
    return;
  }

  const Vec3 tau = ctrl.update(ekf.quaternion(), ekf.bodyRate(), dt);

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
  if (all) addNullSpaceTorque(spd, params::kWheelInertia, params::kNullGain, params::kNullBiasRadS, u);

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
  for (int i = 0; i < kN; i++) wheels[i].updateSpeed(dt);

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
      if (fabsf(wheels[i].speed()) > params::kWheelOverspeedRadS) {
        disarm("WHEEL_OVERSPEED");
        return;
      }
    }
  }

  for (int i = 0; i < kN; i++) {
    if (state == State::Manual) {
      // Ramp the setpoint toward the SPEED target (limits overshoot and current surges).
      const float step = params::kManualAccelRadS2 * dt;
      const float diff = manualTarget[i] - wheelCmd[i];
      wheelCmd[i] += diff > step ? step : (diff < -step ? -step : diff);
      wheels[i].setDuty(
          speedLoops[i].update(wheelCmd[i], wheels[i].speed(), dt, params::kManualFreeSpeedRadS), dt);
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

  if (millis() - calStartMs < kCalDurationMs) return;

  if (calGyroN < kMinCalSamples || calAccelN < kMinCalSamples / 2) {
    Serial.println("# calibration: too few IMU samples, retrying");
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
    Serial.println("# calibration: table is moving, HOLD STILL - restarting");
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
    Serial.println("# calibration: bad gravity vector, retrying");
    resetCalibration();
    return;
  }

  Serial.printf("# calibrated: bias[deg/s]=%.4f,%.4f,%.4f  (N=%lu)\n", mean.x * kRadToDeg,
                mean.y * kRadToDeg, mean.z * kRadToDeg, (unsigned long)calGyroN);
  Serial.println(
      "# t_ms,qw,qx,qy,qz,roll,pitch,yaw,bx,by,bz,sig_roll,sig_pitch,sig_yaw,accel_status,w0,w1,w2,w3,state");
  haveLastGyro = false;
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
  if (!imu.begin(IMU_I2C_BUS, kImuI2cAddr)) {
    Serial.println("# ERROR: BNO085 not found / reports not enabled. Check wiring and address.");
    while (true) {
      stopAllWheels();
      delay(1000);  // halt in setup only; loop() stays non-blocking
    }
  }
  Serial.println("# BNO085 ready. HOLD THE TABLE STILL for calibration.");
  resetCalibration();
  lastGyroMs = millis();
  lastWheelUs = lastLogUs = micros();
}

void loop() {
  pollCommands();  // KILL is handled here, before any IMU work

  // Drain everything the IMU has queued, in arrival order.
  ImuSample s;
  while (imu.poll(s)) {
    if (s.kind == ImuSample::Gyro) latestGyro = s.v;
    else if (s.kind == ImuSample::Accel) latestAccel = s.v;
    else if (s.kind == ImuSample::Mag) latestMag = s.v;  // logged only, never fused

    if (state == State::Calibrating) {
      handleCalibrationSample(s);
      if (s.kind == ImuSample::Gyro) lastGyroMs = millis();
      continue;
    }

    if (s.kind == ImuSample::Gyro) {
      lastGyroMs = millis();
      if (haveLastGyro) {
        const float dt = (uint32_t)(s.t_us - lastGyroUs) * 1.0e-6f;
        if (dt > 0.0f && dt < 0.05f) {  // ignore stalls/glitches
          ekf.predict(s.v, dt);
          if (state == State::Armed) controlStep(dt);
        }
      }
      lastGyroUs = s.t_us;
      haveLastGyro = true;
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

  // Data log runs in every state (calibration included) so each run is fully captured.
  const uint32_t logPeriodUs = (uint32_t)(1.0e6f / params::kLogHz);
  if ((uint32_t)(nowUs - lastLogUs) >= logPeriodUs) {
    lastLogUs += logPeriodUs;
    if ((uint32_t)(nowUs - lastLogUs) >= logPeriodUs) lastLogUs = nowUs;  // fell behind: resync
    logData();
  }
  logger.service();

  if (state != State::Calibrating && millis() - lastUsbMs >= params::kUsbTelemetryMs) {
    lastUsbMs = millis();
    printUsbTelemetry();
  }
}
