#pragma once
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include "quat_math.h"

// Thin wrapper around the Adafruit BNO08x driver. Enables the uncalibrated gyro,
// the accelerometer and the UNCALIBRATED magnetometer, all at 100 Hz. The
// magnetometer is for data logging only: it is NOT fused by the EKF (motor
// interference). The chip's own fusion outputs are not used.

struct ImuSample {
  enum Kind : uint8_t { None, Gyro, Accel, Mag } kind = None;
  Vec3 v{0.0f, 0.0f, 0.0f};  // gyro: rad/s, accel: m/s^2, mag: uT (body frame)
  uint32_t t_us = 0;         // micros() when the report was read on the host
};

class Bno085Imu {
 public:
  explicit Bno085Imu(int8_t resetPin = -1) : bno_(resetPin) {}

  bool begin(TwoWire& wire, uint8_t addr);

  // Returns true and fills `out` if a gyro, accel or mag report was available.
  // Re-enables reports automatically if the sensor reset itself.
  bool poll(ImuSample& out);

 private:
  bool enableReports();

  Adafruit_BNO08x bno_;
  sh2_SensorValue_t value_;
};
