#include "imu_bno085.h"

// Report intervals in microseconds. Sensor collection and control run at 100 Hz.
// The BNO085 cannot run every sensor at its maximum rate at once and I2C bandwidth
// is limited; three reports at 100 Hz is within budget.
static constexpr uint32_t kGyroIntervalUs = 10000;   // 100 Hz (drives EKF predict + control)
static constexpr uint32_t kAccelIntervalUs = 10000;  // 100 Hz
static constexpr uint32_t kMagIntervalUs = 10000;    // 100 Hz (BNO085 max), logging only

// If INT has not been seen for this long, read anyway (INT unwired/broken, or a missed edge).
// 2026-10-20 logging test: lat = 19.5 ms with a 20 ms fallback, i.e. INT assertions were being
// missed. An empty read is cheap (4-byte header), so poll often enough to bound that.
static constexpr uint32_t kFallbackPollUs = 5000;
// An INT edge older than this is stale (already used); fall back to the read time.
static constexpr uint32_t kMaxEdgeAgeUs = 50000;
// Sanity bound on a report's delay relative to its transfer.
static constexpr int32_t kMaxReportAgeUs = 200000;

static constexpr int kBeginAttempts = 3;
// Wait for the chip to boot after a hardware reset.
static constexpr uint32_t kBootWaitMs = 300;
// No sensor report for this long => re-enable reports.
static constexpr uint32_t kNoReportMs = 500;

void Bno085Imu::reenable() {
  lastEnableOk_ = enableReports();
  lastEnableMs_ = millis();
  enableCount_++;
}

// ---- INT (H_INTN) edge capture ----
static volatile uint32_t sEdgeUs = 0;
static volatile bool sEdgeFresh = false;
static volatile uint32_t sEdgeCount = 0;

static void onImuInt() {
  sEdgeUs = micros();
  sEdgeFresh = true;
  sEdgeCount = sEdgeCount + 1;
}

uint32_t Bno085Imu::intCount() const { return sEdgeCount; }

bool Bno085Imu::begin(TwoWire& wire, uint8_t addr) {
  ready_ = false;
  if (intPin_ >= 0) pinMode(intPin_, INPUT);
  for (int attempt = 0; attempt < kBeginAttempts; attempt++) {
    // Hardware reset: a chip left hung by a previous run (e.g. a Teensy upload mid-transfer)
    // won't start otherwise. Fixed boot wait: INT is not driven while the chip boots, so it
    // can't be used to tell when the chip is ready.
    if (resetPin_ >= 0) {
      pinMode(resetPin_, OUTPUT);
      digitalWrite(resetPin_, LOW);
      delay(10);
      digitalWrite(resetPin_, HIGH);
      delay(kBootWaitMs);
    }
    if (!bno_.begin_I2C(addr, &wire)) continue;
    // Replace Adafruit's one-value sensor handler with ours (keeps every report).
    sh2_setSensorCallback(onSensorEvent, this);
    if (!enableReports()) continue;
    head_ = tail_ = 0;
    if (intPin_ >= 0) attachInterrupt(digitalPinToInterrupt(intPin_), onImuInt, FALLING);
    lastServiceUs_ = micros();
    lastEventMs_ = lastEnableMs_ = millis();
    ready_ = true;
    return true;
  }
  return false;
}

bool Bno085Imu::enableReports() {
  const bool wasIn = inLib_;
  inLib_ = true;
  const bool g = bno_.enableReport(SH2_GYROSCOPE_UNCALIBRATED, kGyroIntervalUs);
  const bool a = bno_.enableReport(SH2_ACCELEROMETER, kAccelIntervalUs);
  const bool m = bno_.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, kMagIntervalUs);
  // Turn off the chip's own dynamic calibration and its periodic save of calibration data (DCD)
  // to flash: we don't use them (gyro and mag are read uncalibrated; the EKF estimates gyro bias).
  // This was a guess at the chip's periodic 60-120 ms output stalls (pairs ~22.4 s apart every
  // ~132 s, sequence advancing with nothing sent); STATIC_0009 shows it did NOT remove them.
  // Kept as harmless. Runtime-only settings, so re-applied after every chip reset.
  const bool c = sh2_setCalConfig(0) == SH2_OK;
  const bool d = sh2_setDcdAutoSave(false) == SH2_OK;
  inLib_ = wasIn;
  return g && a && m && c && d;
}

// Read one transfer from the chip if it has data (INT low) or the fallback poll is due.
// Reports in it arrive through onSensorEvent() and are queued.
void Bno085Imu::service() {
  const uint32_t now = micros();
  // Read if INT is low now OR fell since the last read: the chip pulses INT, so a level check
  // alone can miss an assertion that ended before loop() looked.
  if (intPin_ >= 0 && digitalReadFast(intPin_) == HIGH && !sEdgeFresh &&
      (uint32_t)(now - lastServiceUs_) < kFallbackPollUs) {
    return;  // nothing waiting
  }
  noInterrupts();
  const bool fresh = sEdgeFresh;
  const uint32_t edge = sEdgeUs;
  const uint32_t readUs = micros();  // after the edge snapshot, so readUs - edge >= 0
  sEdgeFresh = false;
  interrupts();
  if (fresh && (uint32_t)(readUs - edge) < kMaxEdgeAgeUs) {
    transferUs_ = edge;
    const uint32_t lat = readUs - edge;
    if (lat > maxLatencyUs_) maxLatencyUs_ = lat;
    freshCount_++;
  } else {
    transferUs_ = readUs;
  }
  lastServiceUs_ = now;
  serviceCount_++;
  inLib_ = true;
  sh2_service();
  inLib_ = false;
}

void Bno085Imu::background() {
  if (!ready_ || inLib_) return;
  service();
}

void Bno085Imu::push(const ImuSample& s) {
  const uint8_t next = (uint8_t)((head_ + 1) % kQueueLen);
  if (next == tail_) {
    queueDrops_++;
    return;
  }
  queue_[head_] = s;
  head_ = next;
}

void Bno085Imu::onSensorEvent(void* cookie, sh2_SensorEvent_t* event) {
  Bno085Imu* self = static_cast<Bno085Imu*>(cookie);
  sh2_SensorValue_t v;
  self->eventCount_++;
  self->lastEventMs_ = millis();
  if (sh2_decodeSensorEvent(&v, event) != SH2_OK) return;

  ImuSample s;
  s.seq = v.sequence;
  // The Adafruit I2C HAL passes a host timestamp of 0, so the low 32 bits of timestamp_uS are
  // just the report's time relative to the INT assertion (negative = generated before it).
  const int32_t rel = (int32_t)(uint32_t)event->timestamp_uS;
  s.t_us = self->transferUs_ + ((rel > -kMaxReportAgeUs && rel < kMaxReportAgeUs) ? rel : 0);
  switch (v.sensorId) {
    case SH2_GYROSCOPE_UNCALIBRATED:
      s.kind = ImuSample::Gyro;
      s.v = {v.un.gyroscopeUncal.x, v.un.gyroscopeUncal.y, v.un.gyroscopeUncal.z};
      break;
    case SH2_ACCELEROMETER:
      s.kind = ImuSample::Accel;
      s.v = {v.un.accelerometer.x, v.un.accelerometer.y, v.un.accelerometer.z};
      break;
    case SH2_MAGNETIC_FIELD_UNCALIBRATED:
      s.kind = ImuSample::Mag;
      s.v = {v.un.magneticFieldUncal.x, v.un.magneticFieldUncal.y, v.un.magneticFieldUncal.z};
      break;
    default:
      return;
  }
  self->push(s);
}

bool Bno085Imu::poll(ImuSample& out) {
  if (bno_.wasReset()) {
    resetCount_++;
    reenable();
  }
  // Watchdog: if reports stop (e.g. a chip reset that wiped the configuration and the
  // re-enable did not take), enable them again.
  const uint32_t nowMs = millis();
  if (nowMs - lastEventMs_ > kNoReportMs && nowMs - lastEnableMs_ > kNoReportMs) reenable();
  if (head_ == tail_) service();
  if (head_ == tail_) return false;
  out = queue_[tail_];
  tail_ = (uint8_t)((tail_ + 1) % kQueueLen);
  return true;
}
