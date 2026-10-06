#include "data_logger.h"

#include <string.h>

#include "control_params.h"

static const char kHeader[] =
    "mcutime_ms,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2,"
    "mag_x_uT,mag_y_uT,mag_z_uT,motor_a_measured_rate_rpm,motor_b_measured_rate_rpm,"
    "motor_c_measured_rate_rpm,motor_d_measured_rate_rpm\n";

void DataLogger::say(const char* msg) {
  const size_t n = strlen(msg);
  if ((size_t)radio_.availableForWrite() >= n) radio_.write((const uint8_t*)msg, n);
  Serial.print(msg);
}

bool DataLogger::begin() {
  sd_ = false;
  if (!SD.begin(BUILTIN_SDCARD)) {
    say("# SD: not available (no card?) - radio only\n");
  } else {
    for (int i = 1; i <= 9999; i++) {
      snprintf(name_, sizeof(name_), "LOG_%04d.CSV", i);
      if (!SD.exists(name_)) break;
      if (i == 9999) name_[0] = '\0';  // all names used
    }
    if (name_[0] != '\0') file_ = SD.open(name_, FILE_WRITE);
    if (name_[0] == '\0' || !file_) {
      say("# SD: could not create a log file - radio only\n");
      name_[0] = '\0';
    } else {
      sd_ = true;
      char msg[48];
      snprintf(msg, sizeof(msg), "# SD: logging to %s\n", name_);
      say(msg);
    }
  }

  const size_t h = sizeof(kHeader) - 1;
  if (sd_) {
    memcpy(buf_, kHeader, h);
    len_ = h;
  }
  if ((size_t)radio_.availableForWrite() >= h) radio_.write((const uint8_t*)kHeader, h);
  lastFlushMs_ = millis();
  return sd_;
}

void DataLogger::logRow(const LogRow& r) {
  char line[192];
  const int n = snprintf(
      line, sizeof(line),
      "%lu,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f\n",
      (unsigned long)r.ms, r.gyro.x, r.gyro.y, r.gyro.z, r.accel.x, r.accel.y, r.accel.z, r.mag.x,
      r.mag.y, r.mag.z, r.rpm[0], r.rpm[1], r.rpm[2], r.rpm[3]);
  if (n <= 0 || n >= (int)sizeof(line)) return;

  if (sd_) {
    if (len_ + (size_t)n <= kBufBytes) {
      memcpy(buf_ + len_, line, n);
      len_ += n;
    } else {
      dropped_++;
    }
  }

  if (rowCount_ % params::kXbeeLogDecimation == 0 &&
      (size_t)radio_.availableForWrite() >= (size_t)n) {
    radio_.write((const uint8_t*)line, n);
  }
  rowCount_++;
}

bool DataLogger::writeOut(size_t n) {
  if (n == 0) return true;
  if (file_.write((const uint8_t*)buf_, n) != n) return false;
  memmove(buf_, buf_ + n, len_ - n);
  len_ -= n;
  return true;
}

void DataLogger::failSd() {
  sd_ = false;
  len_ = 0;
  file_.close();
  say("# SD: write failed - logging to radio only\n");
}

void DataLogger::service() {
  if (!sd_) return;

  // At most one chunk per call keeps each call's SD latency bounded.
  if (len_ >= kChunk) {
    if (!writeOut(kChunk)) failSd();
    return;
  }

  if (millis() - lastFlushMs_ >= params::kSdFlushMs) {
    lastFlushMs_ = millis();
    if (!writeOut(len_)) {
      failSd();
      return;
    }
    file_.flush();
  }
}
