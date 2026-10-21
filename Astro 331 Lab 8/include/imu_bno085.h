#pragma once
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include "quat_math.h"

// Wrapper around the Adafruit BNO08x driver. Enables the uncalibrated gyro, the accelerometer
// and the UNCALIBRATED magnetometer, all at 100 Hz. The magnetometer is for data logging only:
// it is NOT fused by the EKF (motor interference). The chip's own fusion outputs are not used.
//
// Differences from the stock Adafruit usage (2026-10-20, INT and RST wired):
//  - RST: the chip is hardware-reset before every begin attempt (fixes the hang after a Teensy
//    upload, which used to need a power cycle).
//  - INT (H_INTN, active low): an interrupt records when the chip asserts it; the chip is only
//    read when INT is low (plus a slow fallback poll in case INT never arrives).
//  - Every report in a transfer is kept. Adafruit's getSensorEvent() keeps only the LAST report
//    of each transfer, and the BNO085 packs reports that fall due together (gyro + accel + mag
//    at 100 Hz) into one transfer, so gyro reports were overwritten: most of the "missed"
//    sequence numbers in the 2026-10-15 static test. Reports now go through our own SH2
//    callback into a small queue.
//  - Timestamps: each report's host time = INT edge time + the report's own delay from the
//    SH2 timebase (100 us resolution), so dt between gyro samples is the chip's, not the
//    host's read jitter. Without a fresh INT edge, the read time is used.

struct ImuSample {
  enum Kind : uint8_t { None, Gyro, Accel, Mag } kind = None;
  Vec3 v{0.0f, 0.0f, 0.0f};  // gyro: rad/s, accel: m/s^2, mag: uT (body frame)
  uint32_t t_us = 0;         // host micros() of the sample (see above)
  // Per-sensor 8-bit report counter set by the BNO085 itself: +1 for every report it sends.
  // A jump > 1 means the chip sent reports the host never received.
  uint8_t seq = 0;
};

class Bno085Imu {
 public:
  Bno085Imu(int8_t resetPin, int8_t intPin) : bno_(-1), resetPin_(resetPin), intPin_(intPin) {}

  // Hardware reset + init, retried a few times. Returns false if the chip never answers.
  bool begin(TwoWire& wire, uint8_t addr);

  // Returns true and fills `out` with the next queued gyro, accel or mag report, reading the
  // chip first if the queue is empty and it has data. Re-enables reports if the chip reset.
  bool poll(ImuSample& out);

  // Read the chip into the queue only (no processing). Safe to call from yield(): does nothing
  // before begin() finishes or while the main code is already inside the SH2 library. Teensy's
  // SdFat calls yield() while it waits on the card, so this keeps the IMU serviced during SD
  // write stalls (up to ~40 ms measured 2026-10-20).
  void background();

  // Diagnostics.
  uint32_t maxLatencyUs() const { return maxLatencyUs_; }  // INT edge -> host read, worst case
  uint32_t queueDrops() const { return queueDrops_; }
  uint32_t intCount() const;                              // INT edges seen (0 => INT not working)
  void clearMaxLatency() { maxLatencyUs_ = 0; }
  // Debug counters: chip reads attempted, sensor events decoded, chip resets seen.
  uint32_t serviceCount() const { return serviceCount_; }
  uint32_t freshCount() const { return freshCount_; }  // reads timed from an INT edge
  uint32_t eventCount() const { return eventCount_; }
  uint32_t resetCount() const { return resetCount_; }
  uint32_t enableCount() const { return enableCount_; }  // re-enables after begin()
  bool lastEnableOk() const { return lastEnableOk_; }

 private:
  bool enableReports();
  void reenable();
  void service();
  void push(const ImuSample& s);
  static void onSensorEvent(void* cookie, sh2_SensorEvent_t* event);

  Adafruit_BNO08x bno_;
  int8_t resetPin_, intPin_;
  uint32_t transferUs_ = 0;   // host time the current transfer's INT was asserted
  uint32_t lastServiceUs_ = 0;
  uint32_t maxLatencyUs_ = 0, queueDrops_ = 0;
  uint32_t serviceCount_ = 0, freshCount_ = 0, eventCount_ = 0, resetCount_ = 0, enableCount_ = 0;
  uint32_t lastEventMs_ = 0, lastEnableMs_ = 0;
  bool lastEnableOk_ = true;

  bool ready_ = false;  // begin() finished
  bool inLib_ = false;  // inside the SH2/Adafruit library (guards re-entry from yield())

  static constexpr uint8_t kQueueLen = 64;  // ~210 ms of gyro+accel+mag at 100 Hz
  ImuSample queue_[kQueueLen];
  uint8_t head_ = 0, tail_ = 0;  // pop at tail_, push at head_
};
