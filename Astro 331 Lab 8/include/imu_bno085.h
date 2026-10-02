#pragma once
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include "quat_math.h"

// Thin wrapper around the Adafruit BNO08x driver. Enables ONLY the uncalibrated
// gyro and the accelerometer; the magnetometer report is deliberately never
// enabled (motor interference) and the chip's own fusion outputs are not used.

struct ImuSample {
  enum Kind : uint8_t { None, Gyro, Accel } kind = None;
  Vec3 v{0.0f, 0.0f, 0.0f};  // gyro: rad/s, accel: m/s^2 (body frame)
  uint32_t t_us = 0;         // micros() when the report was read on the host
};

class Bno085Imu {
 public:
  explicit Bno085Imu(int8_t resetPin = -1) : bno_(resetPin) {}

  bool begin(TwoWire& wire, uint8_t addr);

  // Returns true and fills `out` if a gyro or accel report was available.
  // Re-enables reports automatically if the sensor reset itself.
  bool poll(ImuSample& out);

 private:
  bool enableReports();

  Adafruit_BNO08x bno_;
  sh2_SensorValue_t value_;
};
