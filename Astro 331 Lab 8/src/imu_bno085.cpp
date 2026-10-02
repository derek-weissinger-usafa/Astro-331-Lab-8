#include "imu_bno085.h"

// Report intervals in microseconds. The BNO085 cannot run every sensor at its
// maximum rate at once and I2C bandwidth is limited, so these are conservative.
static constexpr uint32_t kGyroIntervalUs = 5000;    // 200 Hz
static constexpr uint32_t kAccelIntervalUs = 10000;  // 100 Hz

bool Bno085Imu::begin(TwoWire& wire, uint8_t addr) {
  if (!bno_.begin_I2C(addr, &wire)) return false;
  return enableReports();
}

bool Bno085Imu::enableReports() {
  const bool g = bno_.enableReport(SH2_GYROSCOPE_UNCALIBRATED, kGyroIntervalUs);
  const bool a = bno_.enableReport(SH2_ACCELEROMETER, kAccelIntervalUs);
  return g && a;
}

bool Bno085Imu::poll(ImuSample& out) {
  if (bno_.wasReset()) enableReports();
  if (!bno_.getSensorEvent(&value_)) return false;

  const uint32_t now = micros();
  switch (value_.sensorId) {
    case SH2_GYROSCOPE_UNCALIBRATED:
      out.kind = ImuSample::Gyro;
      out.v = {value_.un.gyroscopeUncal.x, value_.un.gyroscopeUncal.y, value_.un.gyroscopeUncal.z};
      out.t_us = now;
      return true;
    case SH2_ACCELEROMETER:
      out.kind = ImuSample::Accel;
      out.v = {value_.un.accelerometer.x, value_.un.accelerometer.y, value_.un.accelerometer.z};
      out.t_us = now;
      return true;
    default:
      return false;
  }
}
