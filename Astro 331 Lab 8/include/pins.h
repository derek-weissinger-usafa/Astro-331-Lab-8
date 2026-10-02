#pragma once
#include <stdint.h>

// TODO: placeholder pin assignments. Replace once the wiring is confirmed.
// Teensy 4.1 I2C buses: Wire (SDA 18, SCL 19), Wire1 (SDA1 17, SCL1 16), Wire2 (SDA2 25, SCL2 24).

#define IMU_I2C_BUS Wire        // TODO: confirm bus (STEMMA QT cable on pins 18/19?)
constexpr int kPinImuReset = -1;   // TODO: BNO085 RST pin, -1 = not connected
constexpr int kPinImuInt = -1;     // TODO: BNO085 INT pin, -1 = not connected
constexpr uint8_t kImuI2cAddr = 0x4A;  // Adafruit BNO085 default; 0x4B if the address jumper is bridged
