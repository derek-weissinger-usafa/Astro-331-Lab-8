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
- `src/imu_bno085.cpp` — Adafruit BNO08x wrapper; enables uncalibrated gyro, accelerometer and **uncalibrated magnetometer**, all at 100 Hz. The gyro sample drives EKF predict + the attitude control step, so **collection and control run at 100 Hz**. The magnetometer is read for the data log only and is never fused.
- `src/data_logger.cpp` — raw data log at **20 Hz** to a new CSV on the built-in SD card each boot (`LOG_0001.CSV`, `LOG_0002.CSV`, … first unused name; no RTC), and every 4th row (5 Hz) in the same format over the XBee. Columns: `mcutime_ms, gyro_x/y/z_rad_s` (uncalibrated, not bias-corrected), `accel_x/y/z_m_s2`, `mag_x/y/z_uT` (uncalibrated), `motor_a..d_measured_rate_rpm`. Runs in every state, calibration included. Non-blocking: RAM buffer → 512-byte SD writes, flush every 1 s (power cut loses ≤ 1 s). Without a card it reports `# SD: not available` and still streams to the XBee.
- **XBee link (`Serial8`, 9600 baud ≈ 960 B/s):** data rows (~86–109 B at 5 Hz ≈ 500 B/s) plus command replies, which start with `#`, `S,` or `PONG` (data rows start with a digit). Serial8's built-in TX buffer is only 40 bytes, so `main.cpp` adds 1 KB with `addMemoryForWrite`. Without that, every line longer than 40 bytes is silently dropped by the non-blocking senders. Raising the XBee data rate above ~8 Hz needs a higher baud.
- `src/main.cpp` — startup: hold table still ~3 s (gyro bias + initial tilt), then run and stream CSV telemetry at 20 Hz.
- **Attitude hold controller** (build-verified, NOT yet run on hardware; all gains/inertias/limits are placeholders in `include/control_params.h`):
  - `include/wheel_geometry.h` — pyramid of 4 wheels, axes 45° above horizontal tilted up toward +z, azimuth A=0°, B=90°, C=180°, D=270° from body +x (CCW from above). Min-norm allocation over *enabled* wheels (`wheelEnabled[]` in `main.cpp` is the fault hook; 3 wheels still span all axes) plus a null-mode speed term for 4 wheels.
  - `src/attitude_ctrl.cpp` — quaternion PD, holds level / yaw 0 (identity reference). Ki defaults to 0 (wheels already integrate constant torque).
  - `src/wheel_motor.cpp`, `include/wheel_speed_ctrl.h` — TB9051FTG dual-PWM drive + encoder speed + 100 Hz PI inner loop (attitude step also 100 Hz) with duty cap and slew limit (shared 5 V rail).
  - `src/command_link.cpp` — ASCII commands on XBee `Serial8` (9600 baud) and USB: `ARM`, `DISARM`/`KILL`, `STATUS`, `PING`, `TEST <A-D> <duty>` (Idle only, 1 s open-loop bench spin), `SPEED <A-D|ALL> <rpm>` (from Idle/Manual → Manual state `M`: closed-loop speed setpoints that ramp to the target at `kManualAccelRadS2`, clamped to ±`kWheelMaxSpeedRadS` ≈ 334 rpm, overspeed disarms; `DISARM`/`KILL` → Idle). XBee telemetry is the data-log stream (see `data_logger.cpp`); the estimator CSV (quaternion, Euler, bias, σ, wheel speeds) goes to USB at 20 Hz.
  - `src/main.cpp` — Calibrating → Idle → Armed (or Idle → Manual via `SPEED`). Auto-disarm on KILL, IMU timeout, |roll/pitch| > 30°, rate > 2 rad/s, wheel overspeed.
  - **Physics limit:** wheels cannot shed momentum, so a constant disturbance torque (e.g. residual imbalance) fills them in ≈ (wheel momentum capacity)/(torque) seconds, after which the table runs away. Capacity depends on the unmeasured wheel inertia and the 5 V speed limit; balance quality sets the hold time.
  - **Per-wheel sign bring-up** (`kMotorSign`, `kEncoderSign`): use `TEST A 0.3` etc. with the table clamped. Measured speed must be positive for positive duty (else flip `kEncoderSign`), and the physical spin must be right-hand about the wheel axis (else flip `kMotorSign` and `kEncoderSign` together).
- `include/pins.h` — confirmed Teensy pin assignments (IMU, XBee, wheel PWM, encoders); see "Teensy pin assignments" below. EKF noise values in `EkfParams` are placeholders to tune from a stationary log.

## Hardware

- MCU: **Teensy 4.1** (Cortex-M7, 600 MHz, hardware FPU — `float` math is cheap; prefer `float` over `double`)
- IMU: **Adafruit BNO085** 9-DoF over STEMMA QT (I2C); does on-chip fusion and can output quaternions
- Wireless: Digi **XBee 3** radios — SparkFun XBee Explorer Dongle (USB) on the ground-station PC, XBee 3 on `Serial8` on the table
- Actuators: 4 reaction wheels, each a **Pololu 10:1 Metal Gearmotor 37D 12 V** (brushed DC)
- Motor drivers: 4× **Pololu TB9051FTG**, two PWM pins each (from past docs; not yet re-verified on the hardware)

### Teensy pin assignments

Transcribed from the team's hand-labeled wiring sheet, `teensy_pin_out.pdf` (repo root), except the encoder pins: the table is actually wired yellow → 6–9 and white → 2–5 (found 2026-10-08 by hand-spin and `TEST` checks; the sheet shows 8–11 / 4–7). Wheels are labeled **A–D**; wire colors are the physical wire colors at the Teensy.

| Wheel | PWM (orange) | PWM (blue) | Encoder ch. A (yellow) | Encoder ch. B (white) |
|---|---|---|---|---|
| A | 22 | 23 | 6 | 2 |
| B | 14 | 15 | 7 | 3 |
| C | 36 | 37 | 8 | 4 |
| D | 24 | 25 | 9 | 5 |

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

Printable bench wiring sheet (overview + every TB9051FTG pin): `power_wiring.html` (repo root; open in a browser).

Batteries:
- **Motors:** the original packs from the past-documentation dissertation — **four packs, each 3× Epoch 18650 (2600 mAh, 8 A max discharge, protected) in series** = 3S: 11.1 V nominal, 12.6 V full, ~9 V empty. One pack per wheel, one per deck corner for mass balance (dissertation Fig 20).
- **Logic:** one **PKCELL ICR18650 10,050 mAh** pack (Adafruit #5035; 3 cells in parallel, 3.7 V nominal, 4.2 V full, 3.0 V protection cutoff, 3 A max, JST-PH lead rated 2 A), near the center under the Teensy.

Two switches: one **motor switch** for all four Epoch packs and one **logic switch** for the PKCELL.

```
Epoch pack A + ──────────── Driver A VIN
Epoch pack B + ──────────── Driver B VIN       (pack + leads stay separate)
Epoch pack C + ──────────── Driver C VIN
Epoch pack D + ──────────── Driver D VIN
Epoch pack A–D − ─ joined ─ MOTOR SWITCH ─ common ground point

PKCELL + ─ LOGIC SWITCH ─ Pololu U3V70F5 5 V boost ─┬─ Teensy VIN
                                                   └─ Driver A–D VCC
PKCELL −, boost GND, driver GNDs, Teensy GND ── common ground point (never tie pack + terminals together)
```

- **Motor switch:** ZF (Cherry) **CR series** rocker — **single-pole**, rated 20 A @ 125 VAC / 16 A @ 250 VAC (no DC rating confirmed; at 12.6 V and a few amps it's well within what an AC switch of that size handles). Because it's single-pole, it switches the **combined negative** of the four motor packs (low-side). Putting it on the + side would tie the four pack + terminals together — don't. Off = pack negatives float, so no motor current flows. Don't connect any motor pack − to ground anywhere except through this switch.
- **Grounding:** bring the motor switch's output, the four driver GNDs, the boost GND and the Teensy GND to one point (star ground), so motor return current doesn't flow through the Teensy's ground wire.
- **Logic switch:** on the PKCELL + lead, ahead of the boost; ≥ 1 A DC is plenty.
- **Power sequence:** logic on first, then motors (so the firmware has driven the PWM pins low before the wheels get power and they don't twitch at boot — `setup()` waits up to 3 s for USB serial before `WheelMotor::begin()` zeroes the PWM pins, so wait a few seconds after logic on before flipping the motor switch); motors off first, then logic.

- **Before connecting any pack:** split each driver's VIN from VCC (the short orange wire ties them today — left tied, 12 V reaches VCC and back-feeds the Teensy VIN), and cut the Teensy's VIN–VUSB trace so the boost doesn't back-feed USB.
- Teensy 4.1 VIN accepts **3.6–5.5 V** (PJRC). The PKCELL spans 3.0–4.2 V, so it's in spec when charged but drops below 3.6 V for the end of its discharge. The team chose to boost it to a 5 V rail with a **Pololu U3V70F5** (2.9–5 V in, 5 V ±4% out, ~7 A continuous input, reverse-voltage protection, UVLO 2.4 V falling, ENABLE pin = true shutdown). Add ~33 µF electrolytic across its VIN/GND to suppress plug-in LC spikes. The board is marked **0J11050**, which every U3V70x variant (5–15 V fixed and 4.5–20 V adjustable) shares — the team's board measured **5.09 V out from 4.15 V in** (no load), confirming the 5 V version. Logic load ≈ 0.25 A at 5 V ≈ 0.4 A from the pack, well inside the 2 A lead; runtime ≈ a day.
- The logic has its own pack, so motor surges can't brown out the Teensy/IMU/XBee (the dissertation ran its Arduino off a motor pack and listed that as a possible cause of its oscillations).
- Packs discharge unevenly, so the same PWM duty gives different wheel torque per pack — close a wheel-speed loop on the encoders rather than commanding raw PWM.
- Optional battery monitoring needs a divider (Teensy analog max 3.3 V): 33 kΩ / 10 kΩ on a motor pack (12.6 V → ≈ 2.9 V), 10 kΩ / 20 kΩ on the PKCELL (4.2 V → 2.8 V). Pin 41/A17 is free.
- Charging: Epoch holders have no balancing — remove cells and charge individually; before a run each pack ≈ 12.4–12.6 V with cells within ~0.05 V. Charge the PKCELL with a single-cell Li-ion charger at ≤ 3 A.

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
