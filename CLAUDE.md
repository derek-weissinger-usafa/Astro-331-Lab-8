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
- `include/pins.h` — confirmed Teensy pin assignments (IMU, XBee, wheel PWM, encoders); see "Teensy pin assignments" below. EKF noise values in `EkfParams` are placeholders to tune from a stationary log.

## Hardware

- MCU: **Teensy 4.1** (Cortex-M7, 600 MHz, hardware FPU — `float` math is cheap; prefer `float` over `double`)
- IMU: **Adafruit BNO085** 9-DoF over STEMMA QT (I2C); does on-chip fusion and can output quaternions
- Wireless: Digi **XBee 3** radios — SparkFun XBee Explorer Dongle (USB) on the ground-station PC, XBee 3 on `Serial8` on the table
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
| 34 (RX8) | white | XBee TX (DOUT) — use `Serial8` |
| 35 (TX8) | blue | XBee RX (DIN) |
| 18 (SDA) | blue | BNO085 SDA — use `Wire`, default address 0x4A |
| 19 (SCL) | yellow | BNO085 SCL |
| VIN / GND / 3.3V | red / green / — | power |
| 12 | — | marked empty |

### Board-side wiring (confirmed by the team)

- **TB9051FTG (×4):** Teensy orange → PWM1, blue → PWM2. OUT1/OUT2 → motor red/black. EN tied high, ENB tied low (always enabled). Short orange wire = power to both VIN and VCC; yellow = GND. **Currently** driver power comes from the **Teensy's VIN pin** (5 V rail), so motors run at ~5 V, not their rated 12 V, and motor current shares the Teensy's supply — being replaced by battery power (see "Power subsystem" below). OCM, DIAG, OCC are not connected (OCC defaults low).
- **Gearmotor encoders (×4):** blue (Vcc) → Teensy 3.3 V, green → GND; yellow (A) / white (B) → Teensy pins above. A/B swing 0–3.3 V, so they're safe for the Teensy with no level shifting. 3.3 V is just below Pololu's 3.5 V minimum encoder supply — if counts are missed or noisy, suspect this first.
- **BNO085:** STEMMA QT red → Teensy 3.3 V, black → GND. I2C only; INT and RST not connected.
- **XBee (table side):** Digi **XBee 3** (non-Pro, ~40 mA TX @ +8 dBm). VCC → Teensy 3.3 V, GND → GND.
- **Teensy 3.3 V rail budget (250 mA max):** XBee 3 (~40 mA TX) + 4 encoders (~40 mA) + BNO085 (tens of mA) ≈ 100–120 mA. Swapping in an XBee 3 **Pro** (~135 mA TX) would leave little margin.

Notes:
- Yellow/white match Pololu's encoder A/B lead colors. Encoder pins aren't all hardware-quadrature (XBAR) capable; use the interrupt-based `Encoder` library (every Teensy 4.1 digital pin has interrupts).

### Power subsystem (planned, not yet wired)

Batteries:
- **Motors:** the original packs from the past-documentation dissertation — **four packs, each 3× Epoch 18650 (2600 mAh, 8 A max discharge, protected) in series** = 3S: 11.1 V nominal, 12.6 V full, ~9 V empty. One pack per wheel, one per deck corner for mass balance (dissertation Fig 20).
- **Logic:** one **PKCELL ICR18650 10,050 mAh** pack (Adafruit #5035; 3 cells in parallel, 3.7 V nominal, 4.2 V full, 3.0 V protection cutoff, 3 A max, JST-PH lead rated 2 A), near the center under the Teensy.

```
Epoch pack A + ─ switch ─ Driver A VIN
Epoch pack B + ─ switch ─ Driver B VIN
Epoch pack C + ─ switch ─ Driver C VIN
Epoch pack D + ─ switch ─ Driver D VIN

PKCELL + ─ switch ─ 5 V boost (PowerBoost 1000C or Pololu S7V8F5) ─┬─ Teensy VIN
                                                                  └─ Driver A–D VCC
All pack negatives, boost GND, driver GNDs, Teensy GND ── common ground (never tie pack + terminals together)
```

- **Before connecting any pack:** split each driver's VIN from VCC (the short orange wire ties them today — left tied, 12 V reaches VCC and back-feeds the Teensy VIN), and cut the Teensy's VIN–VUSB trace so the boost doesn't back-feed USB.
- The PKCELL's 3.0–4.2 V is too low to feed Teensy VIN directly for a steady 3.3 V rail, so it must be **boosted** to 5 V (a buck won't work). Logic load ≈ 0.25 A at 5 V ≈ 0.4 A from the pack, well inside the 2 A lead; runtime ≈ a day.
- The logic has its own pack, so motor surges can't brown out the Teensy/IMU/XBee (the dissertation ran its Arduino off a motor pack and listed that as a possible cause of its oscillations).
- Packs discharge unevenly, so the same PWM duty gives different wheel torque per pack — close a wheel-speed loop on the encoders rather than commanding raw PWM.
- Optional battery monitoring needs a divider (Teensy analog max 3.3 V): 33 kΩ / 10 kΩ on a motor pack (12.6 V → ≈ 2.9 V), 10 kΩ / 20 kΩ on the PKCELL (4.2 V → 2.8 V). Pin 41/A17 is free.
- Charging: Epoch holders have no balancing — remove cells and charge individually; before a run each pack ≈ 12.4–12.6 V with cells within ~0.05 V. Charge the PKCELL with a single-cell Li-ion charger at ≤ 3 A (or in place through the PowerBoost 1000C's USB charger).

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
