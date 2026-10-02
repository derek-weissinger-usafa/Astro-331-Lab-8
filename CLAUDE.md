# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Astro 331 Lab 8 — **3DOF Reaction Wheel Control** on the small air bearing table (team: Gowri Ramaprasad, Derek Weissinger, Owen Crawley). Full brief: `Astro 331 Lab 8 Overview and Requirements.pptx`.

Goals:
- Reaction wheel attitude control in 3 DOF (pitch, yaw, roll).
- Sensor fusion of gyroscopes, magnetometers, and accelerometers to determine attitude in any state.

Requirements:
1. Hold a specified heading with no visible jitter or drift.
2. Determine true attitude to ±1° in all 3 axes.
3. Automatically detect a simulated or real wheel failure and compensate while still meeting 1 and 2.
- Stretch: slew through a series of 4 attitudes along the optimal path, reaching each within ±1° in all 3 axes.

## Layout

The git repo root contains a nested PlatformIO project folder (note the spaces in both paths):

```
Astro 331 Lab 8/                 <- git root (README, CLAUDE.md, lab slides)
└── Astro 331 Lab 8/             <- PlatformIO project
    ├── platformio.ini           <- env: teensy41, framework: arduino
    ├── src/main.cpp             <- firmware entry (setup/loop)
    ├── include/                 <- project headers
    ├── lib/                     <- project-local libraries
    └── test/                    <- PlatformIO unit tests
```

**Status:** Attitude determination is implemented (build-verified, not yet run on hardware). Everything lives in the PlatformIO project folder:
- `include/frames.h` — frame/sign conventions (body z up, right-handed; x=roll, y=pitch, z=yaw; q is inertial→body, Hamilton, scalar-first). Read this before touching estimator or control math.
- `include/quat_math.h` — header-only quaternion + fixed-size `Mat<R,C>` helpers.
- `src/attitude_ekf.cpp` — multiplicative EKF (6-state error: attitude + gyro bias). Gyro propagates; **accelerometer-only** update with magnitude gating. **No magnetometer by design** (motor interference), so yaw is dead-reckoned, relative to power-on, and drifts at the residual gyro-z bias; its σ grows all run.
- `src/imu_bno085.cpp` — Adafruit BNO08x wrapper; enables only uncalibrated gyro (200 Hz) + accelerometer (100 Hz).
- `src/main.cpp` — startup: hold table still ~3 s (gyro bias + initial tilt), then run and stream CSV telemetry at 20 Hz.
- `include/pins.h` — placeholder pins/I2C address (TODO until wiring is confirmed). EKF noise values in `EkfParams` are placeholders to tune from a stationary log.

## Hardware

- MCU: **Teensy 4.1** (Cortex-M7, 600 MHz, hardware FPU — `float` math is cheap; prefer `float` over `double`)
- IMU: **Adafruit BNO085** 9-DoF over STEMMA QT (I2C); does on-chip fusion and can output quaternions
- Wireless: **SparkFun XBee** dongle (XBee radio over serial)
- Actuators: 4 reaction wheels, each a **Pololu 10:1 Metal Gearmotor 37D 12 V** (brushed DC)
- Motor drivers: 4× **Pololu TB9051FTG**, two PWM pins each (from past docs; not yet re-verified on the hardware)

### Teensy pin assignments

Transcribed from the team's hand-labeled wiring sheet, `teensy_pin_out.pdf` (repo root). Wheels are labeled **A–D**; wire colors are the physical wire colors at the Teensy.

| Wheel | PWM (orange) | PWM (blue) | Encoder ch. A (yellow) | Encoder ch. B (white) |
|---|---|---|---|---|
| A | 22 | 23 | 8 | 4 |
| B | 14 | 15 | 9 | 5 |
| C | 36 | 37 | 10 | 6 |
| D | 24 | 25 | 11 | 7 |

| Pin | Wire | Connects to |
|---|---|---|
| 34 (RX8) | white | XBee — use `Serial8` |
| 35 (TX8) | blue | XBee |
| 18 (SDA) | blue | BNO085 SDA — use `Wire`, default address 0x4A |
| 19 (SCL) | yellow | BNO085 SCL |
| VIN / GND / 3.3V | red / green / — | power |
| 12 | — | marked empty |

Notes:
- Which of the orange/blue wires is the driver's PWM1 vs. PWM2 is not recorded — confirm wheel spin direction on the bench.
- Yellow/white match Pololu's encoder A/B lead colors. Encoder pins aren't all hardware-quadrature (XBAR) capable; use the interrupt-based `Encoder` library (every Teensy 4.1 digital pin has interrupts).
- BNO085 wiring isn't on the sheet; it was confirmed by the team (STEMMA QT colors: blue = SDA, yellow = SCL). TB9051FTG OCM/DIAG (useful for wheel-fault detection) aren't wired yet.

## Past Documentation

`Past Documentation/` holds a prior student's work on this table: a paper (`Air Bearing Table Sphere/Dissertation End of Studies Internship.docx`), CAD, and MATLAB/Simulink models (dynamics, motor characterization, linearized saturating-wheel analysis). **Its hardware is outdated** — it used an Arduino MKR 1000 WiFi, ICM-20948 + MPU-6050 IMUs, and Simulink over Wi-Fi. Use it for dynamics, inertia, and motor-characterization reference only; `Gemini Hardware Diagram/old_wiring_diagram.svg` is a rough older wiring layout.

## Build / upload / monitor

`pio` is not on PATH; use the full path. Run from the PlatformIO project folder (`Astro 331 Lab 8/Astro 331 Lab 8`):

```bash
~/.platformio/penv/Scripts/pio.exe run                 # build
~/.platformio/penv/Scripts/pio.exe run -t upload       # flash (Teensy must be connected)
~/.platformio/penv/Scripts/pio.exe device monitor      # serial monitor
~/.platformio/penv/Scripts/pio.exe test -e teensy41    # on-target unit tests
```

Always quote paths — the directory names contain spaces.

## Conventions

- Keep the control loop non-blocking: no `delay()` in `loop()`; use `micros()`/`elapsedMicros` timing so the sensor-fusion and control rates are deterministic.
- Keep sensor drivers, attitude estimation, control law, and wheel fault detection in separate modules so each can be tested independently.
- Angles in code are radians unless a name says otherwise (e.g. `yawDeg`); document frame conventions (body vs. reference) where quaternions/rotations are defined.
