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
//   demo_phase (-/L/U/H/D/S), rate_ref_rad_s, yaw_ref_deg (DEMO SPIN reference; 0 when off),
//   gyro_seq (BNO085 report counter, 0-255), gyro_seq_missed (cumulative reports never received),
//   test_step (MAG MAP step), mag_mode / mag_resid_deg (magnetometer heading aid, mag_aid.h).
//   A row-to-row gyro_seq step > 1 = lost in transfer to the Teensy; an interval > 15 ms with a
//   step of exactly 1 = the BNO085 itself sent late.
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
  int gyroSeq = 0;           // BNO085 gyro report sequence number (0-255, +1 per report sent)
  uint32_t seqMissed = 0;    // cumulative gyro reports sent by the BNO085 but never received
  int testStep = -1;         // MAG MAP schedule step (0-based, see mag_map.h); -1 = not running
  char magMode = '-';        // mag aid mode (MagAid::modeChar): - off, g gated, l/L learning, s/S in use
  float magResidDeg = 0;     // last field heading minus its reference, deg
  char wheelStatus[5] = "....";  // per wheel: . in service, x failed by FAULT TEST (undetected), D dead (detected), - off
  Vec3 errDeg{0, 0, 0};      // controller angle error (deg, body axes); 0 when not Armed
  char faultPhase = '-';     // FAULT TEST: - off, w wait, F failed, h hold, P pass, X fail
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
