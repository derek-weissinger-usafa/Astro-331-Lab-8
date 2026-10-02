#include <Arduino.h>

#include "attitude_ekf.h"
#include "frames.h"
#include "imu_bno085.h"
#include "pins.h"

// Attitude determination: BNO085 gyro + accelerometer -> multiplicative EKF.
// Startup: hold the table still while the gyro bias and initial tilt are measured.

static Bno085Imu imu(kPinImuReset);
static AttitudeEKF ekf;

enum class State { Calibrating, Running };
static State state = State::Calibrating;

// ---- Static calibration accumulators ----
static constexpr uint32_t kCalDurationMs = 3000;
static constexpr float kCalMaxGyroStd = 0.01f;    // rad/s per axis, else "not still"
static constexpr float kCalMaxGyroMean = 0.05f;   // rad/s, rejects a rotating table
static constexpr uint32_t kMinCalSamples = 100;

static uint32_t calStartMs = 0;
static uint32_t calGyroN = 0, calAccelN = 0;
static double gSum[3], gSumSq[3], aSum[3];

static uint32_t lastGyroUs = 0;
static bool haveLastGyro = false;
static uint32_t lastPrintMs = 0;
static AccelResult lastAccelResult = AccelResult::Applied;

static void resetCalibration() {
  calStartMs = millis();
  calGyroN = calAccelN = 0;
  for (int i = 0; i < 3; i++) gSum[i] = gSumSq[i] = aSum[i] = 0.0;
}

static void printTelemetry() {
  const Quat q = ekf.quaternion();
  const Vec3 e = ekf.eulerRad() * kRadToDeg;
  const Vec3 b = ekf.gyroBias() * kRadToDeg;
  const Vec3 s = ekf.sigmaRad() * kRadToDeg;
  Serial.printf("%lu,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d\n",
                millis(), q.w, q.x, q.y, q.z, e.x, e.y, e.z, b.x, b.y, b.z, s.x, s.y, s.z,
                static_cast<int>(lastAccelResult));
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
  // Standard error of the mean bias estimate, worst axis.
  float worst = stdv.x;
  if (stdv.y > worst) worst = stdv.y;
  if (stdv.z > worst) worst = stdv.z;
  const float biasSigma = worst / sqrtf((float)calGyroN);

  if (!ekf.initialize(accelMean, mean, biasSigma)) {
    Serial.println("# calibration: bad gravity vector, retrying");
    resetCalibration();
    return;
  }

  Serial.printf("# calibrated: bias[deg/s]=%.4f,%.4f,%.4f  (N=%lu)\n", mean.x * kRadToDeg,
                mean.y * kRadToDeg, mean.z * kRadToDeg, (unsigned long)calGyroN);
  Serial.println("# t_ms,qw,qx,qy,qz,roll,pitch,yaw,bx,by,bz,sig_roll,sig_pitch,sig_yaw,accel_status");
  haveLastGyro = false;
  state = State::Running;
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  IMU_I2C_BUS.begin();
  IMU_I2C_BUS.setClock(400000);
  if (!imu.begin(IMU_I2C_BUS, kImuI2cAddr)) {
    Serial.println("# ERROR: BNO085 not found / reports not enabled. Check wiring and address.");
    while (true) {
      delay(1000);  // halt in setup only; loop() stays non-blocking
    }
  }
  Serial.println("# BNO085 ready. HOLD THE TABLE STILL for calibration.");
  resetCalibration();
}

void loop() {
  // Drain everything the IMU has queued, in arrival order.
  ImuSample s;
  while (imu.poll(s)) {
    if (state == State::Calibrating) {
      handleCalibrationSample(s);
      continue;
    }

    if (s.kind == ImuSample::Gyro) {
      if (haveLastGyro) {
        const float dt = (uint32_t)(s.t_us - lastGyroUs) * 1.0e-6f;
        if (dt > 0.0f && dt < 0.05f) ekf.predict(s.v, dt);  // ignore stalls/glitches
      }
      lastGyroUs = s.t_us;
      haveLastGyro = true;
    } else if (s.kind == ImuSample::Accel) {
      lastAccelResult = ekf.updateAccel(s.v);
    }
  }

  if (state == State::Running && millis() - lastPrintMs >= 50) {  // 20 Hz telemetry
    lastPrintMs = millis();
    printTelemetry();
  }
}
