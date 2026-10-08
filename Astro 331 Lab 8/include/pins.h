#pragma once
#include <stdint.h>

// Teensy 4.1 pin assignments, from the team wiring sheet (teensy_pin_out.pdf, repo root)
// and confirmed board-side wiring documented in CLAUDE.md.
// Teensy 4.1 I2C buses: Wire (SDA 18, SCL 19), Wire1 (SDA1 17, SCL1 16), Wire2 (SDA2 25, SCL2 24).

// BNO085 IMU: STEMMA QT on Wire (blue SDA -> 18, yellow SCL -> 19), powered from 3.3 V. I2C only.
#define IMU_I2C_BUS Wire
constexpr int kPinImuReset = -1;   // BNO085 RST not connected
constexpr int kPinImuInt = -1;     // BNO085 INT not connected
constexpr uint8_t kImuI2cAddr = 0x4A;  // Adafruit BNO085 default; 0x4B if the address jumper is bridged

// XBee radio on hardware serial 8 (Teensy RX8 34 <- XBee DOUT, TX8 35 -> XBee DIN).
#define XBEE_SERIAL Serial8

// Reaction wheels A-D. Each TB9051FTG has EN/ENB hard-wired to enabled.
// PWM1 high / PWM2 low drives OUT1 high; spin direction vs. body axes is not yet verified.
constexpr int kNumWheels = 4;
constexpr int kPinWheelPwm1[kNumWheels] = {22, 14, 36, 24};  // orange wire -> driver PWM1
constexpr int kPinWheelPwm2[kNumWheels] = {23, 15, 37, 25};  // blue wire   -> driver PWM2

// Wheel encoders (Pololu 64 CPR at motor shaft, 640 CPR at output). Not all are XBAR
// quadrature-capable, so use the interrupt-based Encoder library.
// Encoders are powered from 3.3 V (below Pololu's 3.5 V spec minimum), so A/B are 3.3 V logic.
// As actually wired on the table (differs from teensy_pin_out.pdf, which shows 8-11 / 4-7).
constexpr int kPinWheelEncA[kNumWheels] = {6, 7, 8, 9};  // yellow
constexpr int kPinWheelEncB[kNumWheels] = {2, 3, 4, 5};  // white
