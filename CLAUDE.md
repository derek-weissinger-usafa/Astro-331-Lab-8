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

**Progress record for the team / reports: `PROGRESS_LOG.md`.** **Open issues — see `TODO.md` (prioritised to-do list) and `HANDOFF_2026-10-15.md`:** the table is pendulous; after the balance weights (DEMO_0002) pitch is balanced but roll still rests ~3.3° off level (−y side heavy), so levelling saturates wheel A in ~3 s, wheel A rubs and its pack is ~1 V low, wheel C's high duty/oscillation is unexplained, and wheel speeds logged before 2026-10-15's gear-ratio fix read 1.6× low. Bench scripts are in `tools/` (`speed_sweep.py`, `wheel_hold.py`).

**Status:** Attitude determination is implemented (build-verified, not yet run on hardware). Everything lives in the PlatformIO project folder:
- `include/frames.h` — frame/sign conventions (body z up, right-handed; x=roll, y=pitch, z=yaw; q is inertial→body, Hamilton, scalar-first). Read this before touching estimator or control math.
- `include/quat_math.h` — header-only quaternion + fixed-size `Mat<R,C>` helpers.
- `src/attitude_ekf.cpp` — multiplicative EKF (6-state error: attitude + gyro bias). Gyro propagates; **accelerometer-only** update with magnitude gating. **No magnetometer by design** (motor interference), so yaw is dead-reckoned, relative to power-on, and drifts at the residual gyro-z bias; its σ grows all run.
- `src/imu_bno085.cpp` — Adafruit BNO08x wrapper; enables uncalibrated gyro, accelerometer and **uncalibrated magnetometer**, all at 100 Hz. The gyro sample drives EKF predict + the attitude control step, so **collection and control run at 100 Hz**. The magnetometer is read for the data log only and is never fused.
- `src/data_logger.cpp` — data log, **one row per gyro sample (~100 Hz)**. **No SD file at boot:** sending `BEGIN STATIC TEST [rpm]` (XBee or USB; refused while calibrating) also holds **all four wheels at `kStaticTestWheelRpm` = 100 rpm** via Manual mode (optional rpm argument; `0` = motors off; `DISARM`/`KILL` stops the wheels but logging continues), and opens a new `STATIC_0001.CSV`, `STATIC_0002.CSV`, … (first unused name) and logs every row until power is removed (flush every 1 s, so a power cut loses ≤ 1 s; 32 KB RAM buffer rides out SD stalls; `STATUS` reports `sd=<file>,drop=<n>`). Every 50th row (~2 Hz) streams to the XBee from boot in the same format. Columns: `mcutime_ms` (sample time, µs resolution, 64-bit so it doesn't wrap), `gyro_x/y/z_rad_s` (uncalibrated, not bias-corrected), `accel_x/y/z_m_s2`, `mag_x/y/z_uT` (uncalibrated), `motor_a..d_measured_rate_rpm`, `motor_a..d_duty`, then EKF output: `qw,qx,qy,qz`, `roll/pitch/yaw_deg`, `bias_x/y/z_rad_s`, `sig_roll/pitch/yaw_deg`, `accel_status` (0 applied, 1 gated, 2 singular), `state` (C/I/A/M), demo columns, then `gyro_seq` (the BNO085's own 0–255 report counter) and `gyro_seq_missed` (cumulative reports it sent that the Teensy never got). **IMU sample loss (open issue):** the static test lost ~1/3 of gyro samples and stalled 50–330 ms in ~20 s-spaced clusters (38 gaps > the 100 ms IMU timeout). `STATUS` reports `miss=` (sequence gaps: lost in transfer), `chipgap=` (late with no sequence gap: the BNO085 itself), `maxgap=` (ms), `lat=` (worst INT-to-read delay, ms), `int=` (INT edges), `svc=reads/timed-from-INT`, `ev=` (reports decoded), `rst=`/`en=` (chip resets / report re-enables). **2026-10-20:** most of the loss was the Adafruit library: `getSensorEvent()` keeps only the last report per transfer and the BNO085 packs gyro+accel+mag into one, so gyro reports were overwritten. `imu_bno085.cpp` now uses its own SH2 callback + queue, reads only when INT is low (20 ms fallback poll), timestamps each report from the INT edge + its SH2 delay, hardware-resets the chip via RST before init (no more power cycle after uploads; fixed 300 ms boot wait; the library itself gets no reset pin), and re-enables reports if none arrive for 0.5 s. First check (idle, SD off): `miss=0`, `maxgap=11` ms, `lat` < 0.05 ms. With SD logging on, SD write stalls still blocked `loop()` (5-min static tests: `miss=102, lat=40` ms, then `miss=12, lat=43` after the first fix). Fixes: `main.cpp` overrides the core's weak `yield()` to call `imu.background()` (reads the chip into a 64-deep queue; guarded against re-entry and before `begin()`), the logger skips writes while the card is busy, and the card now runs in **DMA mode** (`SD.sdfs.begin(SdioConfig(DMA_SDIO))`): FIFO mode spins without `yield()` while the card programs flash. Reads are also triggered by any INT edge since the last read (the chip pulses INT; a level check missed some) with a 5 ms fallback poll. **Result, STATIC_0009: 25 of ~29,250 gyro samples lost (0.085%), longest gap 90 ms, no chip resets.** The remainder are BNO085-internal 60–90 ms output stalls (pairs ~22.4 s apart every ~132 s; disabling the chip's calibration/DCD saves did not remove them). The EKF integrates gyro gaps up to the 100 ms IMU timeout (it used to skip anything over 50 ms).
- **XBee link (`Serial8`, 9600 baud ≈ 960 B/s):** data rows (~220–270 B at ~2 Hz ≈ 450–530 B/s) plus command replies, which start with `#`, `S,`, `COUNTS` or `PONG` (data rows start with a digit). Serial8's built-in TX buffer is only 40 bytes, so `main.cpp` adds 1 KB with `addMemoryForWrite`. Without that, every line longer than 40 bytes is silently dropped by the non-blocking senders. Raising the XBee row rate needs a higher baud.
- `src/main.cpp` — startup: hold table still **60 s** (gyro bias + initial tilt; progress messages every 10 s over XBee/USB), then run and stream CSV telemetry at 20 Hz. `CALIBRATE [s]` (Idle only, 10–900 s) redoes it on demand. The z (yaw) bias can only come from this average (gravity carries no heading), and the EKF now **locks the z bias** (`EkfParams::lockZBias`; bias walk 1e-6 x/y, 1e-7 z; floor 2e-5 rad/s). **Warm-up matters most:** the BNO085 z bias shifts ~0.007 °/s (≈ 25 °/h of yaw) over the first ~5 min after power-on, so a calibration at boot is wrong minutes later. Procedure for yaw-sensitive runs: power on, wait ~10 min, `CALIBRATE 180`. Replaying the static logs: calibrating after warm-up gives ~3 °/h (`test_runs/STATIC_motors_off_vs_on_analysis.html`).
- **Attitude hold controller** (build-verified, NOT yet run closed-loop on hardware; gains/limits in `include/control_params.h`). Inertias: table is a MODEL (hollow PLA cube scaled to 5 kg, I = 0.0623 kg·m² each axis) until measured; wheel J = 3.06e-4 kg·m² from CAD (`Past Documentation/CAD files V2/ReactionWheel V2.f3d`, aluminium, 161 g, rim r 4–5 cm × 1.9 cm). **TODO: motor rotor momentum** (≈ N·J_rotor per wheel, a few %) is not modelled yet:
  - `include/wheel_geometry.h` — pyramid of 4 wheels, axes 45° above horizontal tilted up toward +z, azimuth A=0°, B=90°, C=180°, D=270° from body +x (CCW from above). Min-norm allocation over *enabled* wheels (`wheelEnabled[]` in `main.cpp` is the fault hook; 3 wheels still span all axes) plus a null-mode speed term for 4 wheels.
  - `src/attitude_ctrl.cpp` — quaternion PD with trajectory feed-forward: `tau = -Kp th_e - Kd (w - w_ref) + I a_ref + w x (I w + h_wheels)`. `ARM` holds level / yaw 0 (identity, zero rate). Ki defaults to 0 (wheels already integrate constant torque).
  - **`MAG MAP`** (`include/mag_map.h`; XBee/USB, from Idle, **table fixed**): 33 steps × 20 s (~11 min) — all wheels 0/±50/±100/±200 rpm, then each wheel alone ±100/±200 — logging to `MAGMAP_####.CSV` with a `test_step` column, to map the magnetometer offset vs wheel speed/direction. `STOP`/`DISARM`/`KILL` end it; logging continues until power off.
  - `src/demo_spin.cpp` — **`DEMO SPIN`** (XBee/USB, from Idle with wheels stopped and tilt ≤ 30°): Level (reference slews to level at ≤ 0.1 rad/s keeping the current heading, then |roll|,|pitch| < 1° for 3 s; 20 s timeout) → SpinUp (cosine ramp 10 s, CCW about +z) → Hold 10 s → SpinDown 10 s → Settle 5 s → Idle. **Wheel bias:** throughout, the 4-wheel null mode holds A,C at +100 rpm and B,D at −100 rpm (`kDemoWheelBiasRadS`, no net torque/momentum; ramped to 0 during Settle) so no motor shaft sits still. MAG MAP showed stopped shafts leave a random static magnetometer offset (up to ~5 µT / 5° per wheel) and slow wheels can't be filtered; 100 rpm also avoids the ~50 rpm vibration resonance. The z spin drives all wheels the same way, so A and C cross zero briefly (~2 s) on each ramp. Target rate = `0.8·Σ|a_i.z|·J·(60 − bias) rad/s / I_zz`, capped at the requested 30 rpm → **≈ 0.55 rad/s = 5.3 rpm** with the current model (hold wheels ≈ −30 / −50 rad/s). Any wheel > 55 rad/s spins down early. **`STOP`** = controlled spin-down; `KILL` brakes the wheels, which also stops the table abruptly (total momentum ≈ 0). Starts a `DEMO_####.CSV` SD log if none is running; log rows carry `demo_phase` (-/L/U/H/D/S), `rate_ref_rad_s`, `yaw_ref_deg`. Simulated against an independent rigid-body model: levels from 30° in ~11 s, holds 6.36 rpm with < 0.1° tilt.
  - **`DEMO LEVEL`** (same module, level-only mode): the DEMO SPIN Level slew, then hold level at the start heading indefinitely — no spin, no level timeout, **no wheel bias** (full speed range for holding). Phase `L` while levelling, `K` once |roll|,|pitch| < 1° held 3 s (`# DEMO LEVEL: level after N s, holding`). Ends on `STOP`, `KILL`, safety, or a wheel > 55 rad/s; on STOP / wheel limit it reports `armed N s, level held N s, roll, pitch`. Use it to measure how long the current balance can be held.
  - `src/wheel_motor.cpp`, `include/wheel_speed_ctrl.h` — TB9051FTG dual-PWM drive + encoder speed + 100 Hz PI inner loop (attitude step also 100 Hz) with duty cap, slew limit, and a back-EMF current limit (no current sensing: `|duty − speed/free_speed| ≤ I_limit/I_stall`, 2.5 A of a ~5.5 A stall at 12 V). Speed/gain parameters are sized for 12 V packs and the 6.3:1 gearing (free speed ≈ 160 rad/s per unit duty, measured).
  - `src/command_link.cpp` — ASCII commands on XBee `Serial8` (9600 baud) and USB: `ARM`, `DISARM`/`KILL`, `STATUS`, `PING`, `TEST <A-D> <duty>` (Idle only, 1 s open-loop bench spin), `DEMO SPIN`, `DEMO LEVEL`, `COUNTS` (raw encoder counts per wheel since `ZERO`, plus wheel turns at 400 counts/turn; for hand-turn ratio checks), `ZERO` (zero that readout; any state), `SPEED <A-D|ALL> <rpm>` (from Idle/Manual → Manual state `M`: closed-loop speed setpoints that ramp to the target at `kManualAccelRadS2` (speed-loop integrator held while ramping; feedforward `kManualFreeSpeedRadS`; measured speed low-passed by `kManualSpeedLpfTauS`, which USB telemetry shows while in Manual; all Manual-only, Armed unchanged), clamped to ±`kWheelMaxSpeedRadS` (60 rad/s ≈ 573 rpm), overspeed disarms; `DISARM`/`KILL` → Idle). XBee telemetry is the data-log stream (see `data_logger.cpp`); the estimator CSV (quaternion, Euler, bias, σ, wheel speeds, state, then applied duty A–D appended after `state`) goes to USB at 20 Hz.
  - `src/main.cpp` — Calibrating → Idle → Armed (or Idle → Manual via `SPEED`). Auto-disarm on KILL, IMU timeout, |roll/pitch| > 30°, rate > 2 rad/s, wheel overspeed.
  - **Physics limit:** wheels cannot shed momentum, so a constant disturbance torque (e.g. residual imbalance) fills them in ≈ (wheel momentum capacity)/(torque) seconds, after which the table runs away. Capacity depends on the unmeasured wheel inertia and the wheel speed limit (`kWheelMaxSpeedRadS` = 60 rad/s at 12 V); balance quality sets the hold time.
  - **Per-wheel sign bring-up** (`kMotorSign`, `kEncoderSign`): use `TEST A 0.3` etc. with the table clamped. Measured speed must be positive for positive duty (else flip `kEncoderSign`), and the physical spin must be right-hand about the wheel axis (else flip `kMotorSign` and `kEncoderSign` together).
- `include/pins.h` — confirmed Teensy pin assignments (IMU, XBee, wheel PWM, encoders); see "Teensy pin assignments" below. EKF noise values in `EkfParams` are placeholders to tune from a stationary log.

## Hardware

- MCU: **Teensy 4.1** (Cortex-M7, 600 MHz, hardware FPU — `float` math is cheap; prefer `float` over `double`)
- IMU: **Adafruit BNO085** 9-DoF over STEMMA QT (I2C); does on-chip fusion and can output quaternions
- Wireless: Digi **XBee 3** radios — SparkFun XBee Explorer Dongle (USB) on the ground-station PC, XBee 3 on `Serial8` on the table
- Actuators: 4 reaction wheels, each a **Pololu 6.3:1 Metal Gearmotor 37D 12 V** (brushed DC, 64 CPR encoder). All four gearboxes are labeled `GB37-6.3`; confirmed 2026-10-15 by hand-turning each wheel 10 turns with `COUNTS` (~400 counts per wheel turn). The past docs and early firmware assumed 10:1 (640 counts/turn), so **wheel speeds recorded before 2026-10-15's fix read 1.6× low** (multiply old rpm logs by 1.6). Measured ~160–167 rad/s wheel speed per unit duty at 12 V.
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

- **TB9051FTG (×4):** Teensy orange → PWM1, blue → PWM2. OUT1/OUT2 → motor red/black. EN tied high, ENB tied low (always enabled). Yellow = GND. **The firmware is configured for 12 V motors** from the Epoch 3S packs (see "Power subsystem" below); earlier the drivers were powered from the Teensy's 5 V VIN pin. As of 2026-10-06 the new power wiring has **not yet been powered up**. OCM, DIAG, OCC are not connected (OCC defaults low).
- **Gearmotor encoders (×4):** blue (Vcc) → Teensy 3.3 V (re-confirmed 2026-10-15), green → GND; yellow (A) / white (B) → Teensy pins above. A/B swing 0–3.3 V, so they're safe for the Teensy with no level shifting. 3.3 V is just below Pololu's 3.5 V minimum encoder supply — if counts are missed or noisy, suspect this first.
- **BNO085:** STEMMA QT red → Teensy 3.3 V, black → GND, I2C on 18/19. **INT → pin 20, RST → pin 21** (wired 2026-10-20). INT must be a plain `INPUT`: with the Teensy pull-up enabled it never read low and the chip sent no reports.
- **XBee (table side):** Digi **XBee 3** (non-Pro, ~40 mA TX @ +8 dBm). VCC → Teensy 3.3 V, GND → GND.
- **Teensy 3.3 V rail budget (250 mA max):** XBee 3 (~40 mA TX) + 4 encoders (~40 mA) + BNO085 (tens of mA) ≈ 100–120 mA. Swapping in an XBee 3 **Pro** (~135 mA TX) would leave little margin.

Notes:
- Yellow/white match Pololu's encoder A/B lead colors. Encoder pins aren't all hardware-quadrature (XBAR) capable; use the interrupt-based `Encoder` library (every Teensy 4.1 digital pin has interrupts).

### Power subsystem

Firmware is sized for 12 V motor packs, but as of 2026-10-06 nothing has been powered on the new wiring. Confirm the as-built state against the plan below before relying on it.

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
