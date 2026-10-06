#pragma once
#include <Arduino.h>
#include <SD.h>

#include "quat_math.h"

// Raw data log. Each row is written to a new CSV on the built-in SD card
// (LOG_0001.CSV, LOG_0002.CSV, ... first unused name per boot) and every Nth row
// is sent, in the same format, over the radio stream (XBee).
//
// Columns / units:
//   mcutime_ms, gyro_x/y/z (rad/s, uncalibrated, not bias-corrected), accel_x/y/z (m/s^2),
//   mag_x/y/z (uT, uncalibrated), motor_a..d_measured_rate (rpm, filtered encoder speed)
//
// Never blocks loop(): SD data is buffered in RAM and written in 512-byte chunks
// (one per service() call) with a periodic flush; radio rows are dropped if the
// serial TX buffer is full. Rows that don't fit the RAM buffer are counted as dropped.

struct LogRow {
  uint32_t ms = 0;
  Vec3 gyro{0, 0, 0};
  Vec3 accel{0, 0, 0};
  Vec3 mag{0, 0, 0};
  float rpm[4] = {0, 0, 0, 0};
};

class DataLogger {
 public:
  explicit DataLogger(Stream& radio) : radio_(radio) {}

  // Opens the next unused LOG_####.CSV and writes the header to it and the radio.
  // Returns false if SD logging is unavailable (radio rows still go out).
  bool begin();

  void logRow(const LogRow& r);

  // Call every loop(): moves buffered data to the card and flushes periodically.
  void service();

  bool sdActive() const { return sd_; }
  const char* fileName() const { return name_; }
  uint32_t droppedRows() const { return dropped_; }

 private:
  static constexpr size_t kChunk = 512;
  static constexpr size_t kBufBytes = 8192;

  void say(const char* msg);  // status line to radio + USB
  bool writeOut(size_t n);    // write the first n buffered bytes to the card
  void failSd();

  Stream& radio_;
  File file_;
  bool sd_ = false;
  char name_[16] = "";
  char buf_[kBufBytes];
  size_t len_ = 0;
  uint32_t rowCount_ = 0;
  uint32_t dropped_ = 0;
  uint32_t lastFlushMs_ = 0;
};
