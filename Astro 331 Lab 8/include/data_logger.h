#pragma once
#include <Arduino.h>
#include <SD.h>

#include "quat_math.h"

// Data log. One row per gyro sample (~100 Hz). Rows always stream to the radio (XBee) at a
// decimated rate; they are written to the built-in SD card only after startFile(), which is
// triggered by the "BEGIN STATIC TEST" command. No file is created at boot. Logging then runs
// until power is removed (flushed every kSdFlushMs, so at most ~1 s is lost).
//
// Columns / units:
//   mcutime_ms (sample time, us resolution), gyro_x/y/z (rad/s, uncalibrated, not bias-corrected),
//   accel_x/y/z (m/s^2), mag_x/y/z (uT, uncalibrated), motor_a..d_measured_rate (rpm),
//   motor_a..d_duty (applied duty, -1..1), then the attitude estimator (EKF) output:
//   qw,qx,qy,qz (inertial->body), roll/pitch/yaw (deg, 3-2-1), gyro bias x/y/z (rad/s),
//   1-sigma roll/pitch/yaw (deg), accel_status (0 applied, 1 gated, 2 singular), state (C/I/A/M),
//   demo_phase (-/L/U/H/D/S), rate_ref_rad_s, yaw_ref_deg (DEMO SPIN reference; 0 when off).
//
// Never blocks loop(): SD data is buffered in RAM and written in 512-byte chunks (one per
// service() call); radio rows are dropped if the serial TX buffer is full. Rows that don't fit
// the RAM buffer are counted as dropped.

struct LogRow {
  double ms = 0;  // 64-bit-clock sample time; double keeps us resolution over long tests
  Vec3 gyro{0, 0, 0};
  Vec3 accel{0, 0, 0};
  Vec3 mag{0, 0, 0};
  float rpm[4] = {0, 0, 0, 0};
  float duty[4] = {0, 0, 0, 0};
  Quat q{1, 0, 0, 0};
  Vec3 eulerDeg{0, 0, 0};
  Vec3 bias{0, 0, 0};
  Vec3 sigmaDeg{0, 0, 0};
  int accelStatus = 0;
  char state = '?';
  char demoPhase = '-';  // DEMO SPIN phase: - off, L level, U spin-up, H hold, D spin-down, S settle
  float rateRef = 0;     // demo reference yaw rate, rad/s
  float yawRefDeg = 0;   // demo reference yaw, deg
};

class DataLogger {
 public:
  explicit DataLogger(Stream& radio) : radio_(radio) {}

  // Checks for a card (no file is created) and sends the column header to the radio.
  // Returns true if a card is present.
  bool begin();

  // Opens the next unused <prefix>_####.CSV and starts SD logging. Returns false if no card
  // or the file can't be created. Has no effect (returns true) if SD logging is already running.
  bool startFile(const char* prefix);

  void logRow(const LogRow& r);

  // Call every loop(): moves buffered data to the card and flushes periodically.
  void service();

  bool cardPresent() const { return card_; }
  bool sdActive() const { return sd_; }
  const char* fileName() const { return name_; }
  uint32_t droppedRows() const { return dropped_; }

 private:
  static constexpr size_t kChunk = 512;
  static constexpr size_t kBufBytes = 32768;  // ~1.3 s of 100 Hz rows; rides out SD write stalls

  void say(const char* msg);  // status line to radio + USB
  bool writeOut(size_t n);    // write the first n buffered bytes to the card
  void failSd();

  Stream& radio_;
  File file_;
  bool card_ = false;
  bool sd_ = false;
  char name_[24] = "";
  char buf_[kBufBytes];
  size_t len_ = 0;
  uint32_t rowCount_ = 0;
  uint32_t dropped_ = 0;
  uint32_t lastFlushMs_ = 0;
};
