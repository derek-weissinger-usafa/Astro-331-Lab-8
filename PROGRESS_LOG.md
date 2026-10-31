# Lab 8 progress log

Running record of what has been built, tested and found, for the team and for the progress
reports. Newest work is at the bottom of §2. Firmware details and commands live in `CLAUDE.md`,
the prioritised to-do list in `TODO.md`, and the 2026-10-15 bench notes in `HANDOFF_2026-10-15.md`.
Data and analysis pages are in `test_runs/` (index in §5).

Last updated 2026-10-21.

---

## 1. Status against the requirements

| Requirement | Status |
|---|---|
| 1. Hold a heading, no jitter or drift | Controller written and simulated; **not yet run closed-loop on the table**. Blocked by the gravity finding in §2.7: the table is bottom-heavy and rests ~3° off level, so the wheels can only hold level for ~1 s. Heading (yaw) has no gravity torque and is not blocked. |
| 2. Attitude to ±1° in all axes | Roll/pitch: accelerometer-aided EKF, well within ±1° when static. Yaw: gyro-only (magnetometer excluded), drift ~3 °/h after warm-up + 180 s calibration (replay). IMU data path now reliable (§2.8). |
| 3. Detect and compensate a wheel failure | `FAULT TEST` first draft (§2.14): dead-motor detection, 3-wheel control, graded test. Simulated 9/9 PASS; not yet run on the table. |
| Stretch: 4-attitude slew | `SLEW TEST` firmware first draft (§2.13): build-verified and simulated, not yet run on the table. |

---

## 2. Work completed (chronological)

### 2.1 Project setup and hardware identification (2026-09-29 → 10-02)
- PlatformIO project for the **Teensy 4.1**; `CLAUDE.md` documents hardware, pins, wiring, build steps.
- Identified the real hardware: Teensy 4.1, Adafruit **BNO085** IMU (I2C, STEMMA QT), Digi **XBee 3** radio,
  4× Pololu 37D gearmotors with TB9051FTG drivers. The past student's documentation (Arduino MKR1000,
  ICM-20948/MPU-6050, Simulink over Wi-Fi) is **outdated hardware**; it is used only for dynamics and
  motor-characterisation reference.
- Owen transcribed and confirmed the Teensy pin assignments (`teensy_pin_out.pdf`, `include/pins.h`).

### 2.2 Attitude determination (2026-10-02)
- **Multiplicative EKF** (`src/attitude_ekf.cpp`): 6-state error (attitude + gyro bias), gyro propagation,
  accelerometer update with magnitude gating. Frame: body z up, right-handed; x = roll, y = pitch, z = yaw.
- **Magnetometer deliberately excluded** (motor interference), so yaw is relative to power-on and drifts at
  the residual gyro-z bias. Magnetometer is logged for analysis only.

### 2.3 Power system and controller (2026-10-06 → 10-13)
- Owen: power plan and wiring (four 3S Epoch 18650 motor packs, one per wheel; separate PKCELL logic pack with
  a U3V70F5 5 V boost, verified 5.09 V; low-side motor switch; star ground; bench sheet `power_wiring.html`).
- **Attitude controller** (`src/attitude_ctrl.cpp`): quaternion PD with trajectory feed-forward and gyroscopic
  compensation. **Wheel geometry/allocation** (`include/wheel_geometry.h`): 4-wheel pyramid at 45°,
  minimum-norm allocation over the enabled wheels plus a null-mode speed term.
- **Wheel speed loop** (`src/wheel_motor.cpp`): encoder speed + 100 Hz PI, duty cap, slew limit, and a
  back-EMF current limit (no current sensor).
- **Data logger** (`src/data_logger.cpp`): CSV rows to SD and XBee. Found and fixed a silent bug: the
  Teensy's Serial8 TX buffer is 40 bytes, so every XBee line longer than that was dropped
  (`addMemoryForWrite`).
- Owen: encoder pins corrected to the as-built wiring, encoder signs, `SPEED` manual closed-loop mode with
  ramping and low-pass.

### 2.4 Bench session (2026-10-15) — see `HANDOFF_2026-10-15.md`
- **Gear ratio is 6.3:1, not 10:1** (gearbox labels + hand-turn `COUNTS`: ~400 counts per wheel turn).
  Wheel speeds logged before this fix read **1.6× low**.
- Measured free speed ~160–167 rad/s per unit duty at 12 V; speed-loop gains rescaled; 12 V settings.
- Added `COUNTS`/`ZERO` commands, duty telemetry, bench scripts (`tools/speed_sweep.py`, `tools/wheel_hold.py`).
- Findings: B and D healthy; **wheel A rubs** and its pack is ~1 V low; **wheel C** needs high duty and
  oscillates (cause still unknown).

### 2.5 Static IMU test #1 (2026-10-15) — `test_runs/STATIC_0001_analysis.html`
- `BEGIN STATIC TEST` SD logging added.
- **Yaw drift −27 °/h** with the original 3 s gyro calibration → calibration lengthened to **60 s**
  (`CALIBRATE [s]` added).
- Gyro z noise (Allan variance): ARW 2.17e-4 rad/s/√Hz, bias-instability floor ~2 °/h.
- Discovered **IMU sample loss**: ~1/3 of gyro samples missing and 38 freezes > 100 ms (fixed in §2.8).
- Accelerometer magnitude reads 0.981 g (small scale error; harmless, the filter only uses direction).

### 2.6 Motors-on static test and magnetometer map (2026-10-16)
- `test_runs/STATIC_motors_off_vs_on_analysis.html`: the BNO085 **z-gyro bias shifts ~0.007 °/s (~25 °/h of yaw)
  over the first ~5 min after power-on**. Procedure for yaw-sensitive runs: power on, wait ~10 min, `CALIBRATE 180`.
  EKF bias tuning tightened and the z bias locked to the calibration → replay drift **~3 °/h**.
- `MAG MAP` test (`test_runs/MAGMAP_0001_analysis.html`): spinning motors add fast (filterable) magnetometer noise;
  a **stopped** wheel leaves a static offset that depends on where its shaft stops (up to ~5 µT ≈ 5° of heading per
  wheel, ~9 µT for all four); vibration resonance near 50 rpm.
- `DEMO SPIN` (level, then spin about +z) now holds a **100 rpm null-mode wheel bias** so no shaft sits still.

### 2.7 First `DEMO SPIN` on the air bearing (2026-10-16) — `test_runs/DEMO_0001.CSV`
- Three attempts, each stopped by `DEMO_WHEEL_LIMIT` within 1–2 s of the levelling phase.
- **Root cause: the table is a pendulum.** When left alone it rocks in roll/pitch with a ~1.8 s period
  (ωn ≈ 3.3–3.5 rad/s, almost undamped), about a resting tilt of ~3° roll / 1–2° pitch. The centre of mass is
  below the bearing's centre of rotation and slightly off to one side.
  - Gravity stiffness ≈ I·ωn² ≈ **0.9 N·m/rad (~16 mN·m per degree)**. Holding level 3° from rest needs ~50 mN·m;
    the wheels store only ~24 mN·m·s about x, so they fill in ~0.5 s. The controller stiffness (0.25 N·m/rad) is
    weaker than gravity.
  - A pendulum + momentum fit of attempts 2–3 **verified the wheel geometry and signs** on all three axes and gave
    inertia estimates of **0.045–0.13 kg·m² (x/y)** and **0.074–0.082 kg·m² (z)** vs the 0.0623 model.
  - Attempt 1: the table did not respond at all (perfectly still for 35 s, no yaw drift), unlike every later
    idle period — it was probably held or not floating.
- Proposed fixes, **deferred by the team**: trim the balance so the resting attitude is level; change `DEMO SPIN`
  to hold the natural resting tilt (wheels only damp the swing) and spin about the true vertical; add a `BALANCE`
  command that reports the resting tilt and gravity torque for trimming.

### 2.8 IMU reliability: INT/RST wiring and sample-loss fixes (2026-10-20)
Wiring: BNO085 **INT → Teensy pin 20, RST → pin 21**. (During the work a loose ground left the breakout
unpowered; symptom: green LED blinking, nothing on the I2C bus.)

| Test | Gyro samples lost | Longest gap | Gaps > 100 ms |
|---|---|---|---|
| STATIC_0001 (before) | ~33% | 330 ms | 38 |
| SD logging on, first INT build | 102 in ~5 min | — (lat 40 ms) | — |
| + IMU serviced in `yield()` | 12 in ~5 min | — (lat 43 ms) | — |
| STATIC_0008 (+ SD DMA mode) | 370 (1.0%), 314 of them from 2 chip resets | 800 ms | 3 |
| **STATIC_0009 (current)** | **25 of ~29,250 (0.085%)** | **90 ms** | **0** |

What changed (`src/imu_bno085.cpp`, `src/main.cpp`, `src/data_logger.cpp`):
1. **Library bug (the main cause of the old ~1/3 loss):** Adafruit's `getSensorEvent()` keeps only the last
   report of each I2C transfer, and the BNO085 packs gyro + accel + mag into one transfer, so gyro reports were
   overwritten. The driver now registers its own SH2 callback and queues every report.
2. **RST:** the firmware hardware-resets the IMU before every start. Uploads no longer need a power cycle.
3. **INT:** the IMU is read when INT falls (edge captured in an interrupt), with a 5 ms backup poll.
   **INT must be a plain input** — with the Teensy's internal pull-up the line never read low and the chip sent
   nothing. Each sample is timestamped from the INT edge plus the chip's own report delay.
4. **SD stalls:** SD writes blocked `loop()` for up to ~40 ms. The firmware now reads the IMU inside `yield()`
   (which the SD driver calls while waiting), the SD card runs in **DMA mode** (FIFO mode busy-waits without
   yielding), and the logger skips a write while the card is busy.
5. **Remaining gaps are inside the BNO085:** 60–90 ms output stalls in pairs ~22.4 s apart, repeating every
   ~132 s (same pattern in STATIC_0008 and 0009). Disabling the chip's own calibration and calibration saves did
   not remove them; cause unknown. They are below the 100 ms IMU timeout, so they don't disarm the wheels.
6. **EKF across gaps:** the filter used to skip any gyro interval > 50 ms, losing the rotation during a stall
   (~3° of yaw per stall while spinning at 0.55 rad/s). It now integrates gaps up to the 100 ms IMU timeout.
7. **Diagnostics:** `STATUS` reports `miss` (lost samples), `lat` (worst INT-to-read delay, ms), `int`, `svc`,
   `ev`, `rst` (chip resets; 1 = the boot reset), `en`. If the IMU is missing at boot the firmware retries every
   second and prints which I2C addresses answered.

### 2.9 Preliminary data review deck (2026-10-21)
- `Reports/Astro 331 Lab 8_2_3 Body Problem_Prelim_Pres_FILLED.pptx`: 16 slides built on the team template
  (requirements verification table, theory, build progress, 5 data slides with Measure/Compare/Explain/Next,
  future plans, GenAI documentation). Figures and diagrams are in `Reports/figures/` for reuse in the final report.
- To confirm before submitting: slide 4 pass criteria (proposed thresholds), slide 16 GenAI format, Gowri's
  contributions, "2025 Fall" in the slide master, and adding a hardware photo.

### 2.10 DEMO SPIN after balancing, and DEMO LEVEL (2026-10-21)
- The team added balance weights. `test_runs/DEMO_0002.CSV`: six DEMO SPIN attempts. Every one stayed in the
  Level phase (the spin never started) and stopped when wheel A reached the 55 rad/s limit, 2.2–3.7 s after ARM.
- Roll came down from 4–7° to ~1–2° in ~1.3 s (looked level) but settled at ~2°, outside the ±1° needed to start
  the spin. Holding 2° takes ~8 mN·m against gravity, which fills wheel A in ~3 s; the stop then brakes the wheels
  and kicks the table (~8° roll swing).
- The weights worked on pitch and stiffness: free-swing period 1.8 → 3.3 s (gravity's pull ~3× weaker), resting
  pitch ≈ −0.3°. Roll still rests ≈ 3.3° (−y side heavy). Next: trim roll toward +y.
- New **`DEMO LEVEL`** command: level, then hold level with no spin and no wheel bias until STOP / wheel limit;
  reports the hold time. Prelim deck: new slide 14 "Data: DEMO SPIN After Balancing" (`Reports/figures/fig6_demo2_balanced.png`).

### 2.11 DEMO LEVEL tests after the roll trim — `test_runs/DEMO_LEVEL_analysis.html`
- `test_runs/DEMO_0003_OLD.CSV` (saved on the SD card as DEMO_0002; renamed 2026-10-31): five `DEMO LEVEL` runs. All reached level
  (±1° for 3 s) 5–6 s after the command and held steadily (roll/pitch σ ≤ 0.07°, heading within 0.5°) for
  2.0, 5.4, 7.0, 6.9 and **15.8 s** (20.8 s armed) before wheel B hit 55 rad/s.
- Leftover gravity torque fell from ≈ 8 mN·m (roll) to 1.3–3.3 mN·m (now pitch), so the pitch wheel (B) saturates.
- The tilt held while "level" (0.3–0.8° pitch, ≤ 0.2° roll, IMU frame) equals torque ÷ Kp: the expected PD
  steady-state error. A visibly larger deck tilt would be IMU mounting offset (to measure with a bubble level).
- Yaw disturbance ≈ 0.12 mN·m → ≈ 6–7 min of heading hold if roll/pitch hold the table's natural rest tilt
  (now within ≈ 0.5° of level) instead of exact level. Figure: `Reports/figures/fig7_demo_level.png`.
- **Firmware change (2026-10-27):** `DEMO LEVEL` now holds the natural rest tilt: heading held with the full PD,
  roll/pitch only damped (reference tilt = measured tilt), so gravity sets the tilt and the wheels carry no
  steady roll/pitch torque. Chosen over a slowly-tracking reference after a single-axis simulation: pure
  damping stored ~8 mN·m·s for a 2° start vs ~42 mN·m·s with 4 s tracking. Not yet run on the table.

---

### 2.12 Magnetometer calibration and yaw aid (2026-10-27) — `Reports/Magnetometer_Yaw_Aid_Report_2026-10-27.md`
- Fixed-table tests (raw 100 Hz USB stream): a stopped wheel's field depends on its shaft angle (1–5 µT ≈ 3–13°);
  a spinning wheel's mean is repeatable and nearly speed-independent, but reversing it shifts the mean 0.6–1.6 µT.
  Per-wheel direction/speed model fitted for A–C (≤ 0.8° per wheel).
- Firmware: heading-only magnetometer EKF update (also estimates the z gyro bias), `mag_aid` module, `MAG ON/OFF`,
  `KICK`, `RAW ON/OFF`, `WHEEL x ON|OFF`, automatic encoder-fault wheel disable.
- Result: wheels stopped, a 10° yaw error decays with τ ≈ 25 s; wheels spinning, not yet within ±1°
  (gating flicker, 1.4–2.9° error in mixed-direction combos).
- **Wheel D's encoder has a dead channel** (D spun ~500 rpm reading 0). Repair before 4-wheel tests.
- 2026-10-29: aid fixes (speed hysteresis, one reference per wheel-direction pattern, learning only while the
  estimate is fresh). Test 5 (spinning, wheels A+B; C wouldn't start, D off): yaw within ±0.94° in un-kicked
  combos, kicks recover while spinning (τ ≈ 25 s) with ~0.9° left after a kick. 3/4-wheel spinning untested.
- **2026-10-29: Teensy lost power after test 5** — boost outputs 5 V but no 5 V at the Teensy VIN pin, no
  3.3 V, no LEDs, not seen on USB. Blocks all testing; see report §1.7 and `TODO.md` §0.
- **2026-10-30: resolved with the spare Teensy.** The old board's 3.3 V rail was shorted on the board itself
  (0.8 Ω bare, very hot when powered): it is dead. No wiring cause found (driver VIN–VCC / VM–VCC and every
  motor-lead-to-encoder check read open). The spare was flashed (5ec01a5) and brought up
  in stages with 3.3 V and heat checks: IMU clean after reseating its wiring (1 reset, 0 lost samples), XBee OK,
  encoders A–C OK, `TEST x 0.3` A +45.0 / B +45.7 / C +37.5 rad/s (C ~17% slow). Its VIN–VUSB pads are still
  joined (the cut didn't separate them), so uploads are done over USB with the battery off and tests are run over
  the XBee (procedure in `CLAUDE.md`, "Uploading and running").
- **2026-10-30: wheel D's encoder** — a wire was almost pulled out. Reseated, `TEST D ±0.3` gives +44.9 / −46.8 rad/s,
  but it failed again ~30 min later and needed a second reseat: the connection is **intermittent** and must be soldered
  or crimped with strain relief. Wheel C's slow first start (37.5 rad/s) did not repeat: pack C 12.3 V, retests
  46.2–46.5 rad/s, same breakaway (0.08–0.10 duty) as A and B. Full write-up: `Reports/Teensy_Recovery_2026-10-30.html`.
- **2026-10-30: magnetometer wheel model recalibrated with all four wheels** (`MAG MAP`, fixed table, run over the XBee;
  `test_runs/MAGMAP_0003_analysis.html`). A–C repeat Owen's 2026-10-27 values within ~0.15 µT; wheel D measured for the
  first time; speed hardly matters; all four reversing together adds up within 0.2–0.45°. New constants in `mag_aid.cpp`
  (built, not yet uploaded). The raw field now carries a large table-fixed offset (|B| 64 µT vs ~52 µT Earth), so a
  hard-iron calibration is needed before tests where the table yaws.

### 2.13 SLEW TEST, stretch requirement (2026-10-29, first draft)
- New `SLEW TEST` command (`slew_test.h/.cpp`): four target attitudes, shortest-rotation (eigenaxis) slews on a
  smooth profile from any starting attitude, reach each within ±1° in roll/pitch/yaw and hold 5 s, PASS/FAIL per
  target. After the 4th target it keeps holding it until `STOP`. The tilt safety limit is 25° while it runs (the
  bubble has no top). `SLEW SET` / `SLEW LIST` change and show targets. Build-verified; not yet run on the table.
- 3-D simulation: balanced table 4/4 PASS (worst error ≈ 0.02°). The current pendulous table saturates a wheel
  2.3 s into the first slew; yaw-only targets at the natural rest tilt pass 4/4. The default ±10° tilts need the
  free-swing period ≳ 55 s (gravity stiffness ≤ 0.001 N·m/rad, CG within ~0.02 mm of the bearing centre).

### 2.14 FAULT TEST, requirement 3 (2026-10-29, first draft)
- **Detection** (`wheel_fault.h/.cpp`, always on while Armed): a motor driven hard (effort = duty − speed/160
  ≥ 0.2) whose wheel gains < 25% of the predicted speed for 0.2 s is declared dead, taken out of service and
  coasted at the zero-current duty (EN/ENB are hard-wired, so plain "off" would brake it and dump its momentum
  into the table). The other three wheels carry on with whatever is running; allocation already handled 3 wheels.
- **Test:** `FAULT TEST <A-D> [s]` kills that motor's output after the delay without telling the controller
  (the rest of the firmware still sees the commanded duty, like an open lead), with the wheels idling at ±100 rpm
  in the null mode so the failure shows up at once. Graded: right wheel detected within 1 s, back within ±1° within
  10 s, then ±1° for 30 s. `FAULT TEST REAL` grades a failure you cause yourself (pull a lead); `FAULT CLEAR` undoes it.
  Logs gain `wheel_status`, `err_roll/pitch/yaw_deg` (controller error) and `fault_phase`.
- **Motor-level simulation** (back-EMF, friction, pack differences, encoder quantisation, firmware speed loop):
  9/9 graded runs PASS: each wheel during DEMO LEVEL on today's pendulous table, mid-slew and mid-hold in SLEW TEST,
  and an unplugged motor. Detected 0.6–0.8 s after the failure, peak error ≤ 1.0°. 0 false alarms in 4 healthy
  runs. SLEW TEST re-simulated with the same model: balanced 4/4 PASS (≤ 0.08°); today's table with ±10° targets
  stops at the wheel limit 4.9 s in; yaw-only targets at the rest tilt pass 4/4.
  Deliverable: `Reports/Slew_and_Fault_Simulation.html`.

### 2.15 MAG TEST, fixed table, all four wheels (2026-10-30) — `test_runs/MAGTEST_0001_report.html`
- Run over the XBee: ~10 min warm-up, `CALIBRATE 180`, `MAG TEST` (37 graded steps, 57 min: stopped, 14 wheel direction
  patterns at 100–300 rpm, three ±10° estimate kicks, 30 min drift run). Log `test_runs/MAGTEST_0001.CSV`.
- **On board: 36/37 PASS.** Every step stayed within ±1° except step 36 (drift run, all wheels 300 rpm, pattern ++--): 1.02°.
  Kicks pulled back in 73–84 s. Aid in use 78–100% of each step (92% overall). Drift run: max 1.02°, trend −0.40 °/h.
- **Without the aid** (same log replayed on the PC with `MAG OFF`): yaw drifts **5.7 °/h** even after the warm-up and the
  180 s calibration, so it would leave ±1° after ~12 min.
- **Cause of the miss (log replay, `tools/replay`):** each new direction pattern's reference was learned at the yaw error
  of that moment, so errors random-walk from pattern to pattern (step 36 went in at −0.6°; its pattern was learned at
  200 rpm, and at 300 rpm the field sat a further +0.43° off). Replaying the same log with **predicted** references
  (each new pattern's reference from a known one via the MAGMAP_0003 wheel model, fitted on a different run) passes
  **37/37, worst 0.77°**, 0.25° rms vs 0.33°. Also tried and rejected: gating above 250 rpm (gyro gaps let drift in, 1.29°),
  slower correction σ 0.5 (13 fails, slow kick recovery), no stopped re-learn (no gain). σ 0.2 with prediction: 37/37,
  worst 0.67°, kicks back in 45–51 s — tuned on this same log, so try it on the table with `SET MAG_SIGMA 0.2`.
- **Firmware change:** prediction is now the default (`kMagPredictNewPatterns`, `mag_aid.h`); uploaded 2026-10-31, not yet run on the table.
- **Data loss:** 1,529 SD rows lost, all within one 4-min window (10–15 min in, steps 10–13) in short gaps ≤ 17 rows /
  249 ms: SD-card write stalls overflowing the 32 KB buffer; estimation on board was unaffected. 354 IMU samples missed
  (0.1%, steady ~30 per 5 min): the known BNO085 behaviour (STATIC_0009: 0.085%).
- The replay reproduces the logged yaw to 0.047° rms (0.18° max) over the whole run, aid included.

### 2.16 MAG TEST rerun with predicted references (2026-10-31) — `test_runs/MAGTEST_0002_report.html`
- Firmware with prediction as the default; ~3 min after power-on (board warm from the upload), `CALIBRATE 180`
  (z bias −0.0333 °/s), `MAG TEST`. **Stopped at step 15: 10/15 PASS**: steps 8, 9, 11, 12, 15 at 1.02–1.21°, all on or
  right after first visits to new wheel patterns (−−−−, +−−−, −−++).
- **Prediction failed this time.** Per pattern, the wheel-corrected field sat up to **+1.24° (−−−−) and +1.07° (+−−−)** off
  the ++++ reference (run 1: −0.1…+0.4°), growing with the number of reversed wheels: the reversal model moved ~1° between
  sessions. The raw field strength also differed (71.1 µT in run 1, 64.7 µT now; MAGMAP_0003 64.0): the magnetic
  surroundings changed between the runs (something magnetised near the IMU, or the table placed/turned differently).
- **Replays of both logs** (learned / predicted, worst outside kicks): run 1 1.02° / 0.77°, run 2 0.78° / 1.49°. Blends of
  the two, faster correction (σ 0.2) and no stopped re-learn were all inconsistent across the two runs (worst 0.8–1.3°).
  Each new pattern's reference carries 0.3–1.2° of error either way: that is the floor of this magnetometer placement.
- **Firmware:** back to learning new patterns in place (`kMagPredictNewPatterns = false`; prediction only when the aid is
  stale). Heading holds change pattern only once or twice (wheel-bias planner), where learning's random walk barely
  starts. **SD logger:** rows were again lost in a 5–6 min burst (2,398 rows, 7–13 min into the file). The logger now writes
  whole, sector-aligned blocks of up to 8 KB (it wrote one 512 B sector per call, and the 1 s flush left the file
  misaligned) and the RAM buffer is 96 KB (was 32 KB). Uploaded 2026-10-31, not yet tested on a long log.
- Moving the magnetometer away from the wheels (fields fall as 1/r³) would lower that floor, but the team can't change the
  IMU mounting, so it stays a fixed limit: keep pattern changes few during holds.
- **Final presentation not yet updated for this run** (deferred): it still shows the run-1 prediction replay as the
  current firmware (TODO.md).

### 2.17 DEMO LEVEL heading holds on the air bearing, aid on vs off (2026-10-31) — `test_runs/DEMO_0003.CSV`
(The 2026-10-21 DEMO LEVEL log that used this name is now `DEMO_0003_OLD.CSV`.) ~10 min warm-up, `CALIBRATE 180` on the
stand (z bias −0.0379 °/s), table floated, then two holds over the XBee; wheel bias on (100 rpm, `null_bias.h`).
- **Test 1, aid on (214 s):** settled at the rest tilt in 6.3 s (roll 0.89°, pitch −2.77°). Yaw estimate held ±0.2°
  (controller error 0.02° rms). The aid was in use 94% of the time (one +−+− reference learned, gated 1%), field residual
  +0.20° mean / 0.32° rms. Wheels ≤ 34 rad/s, no pattern changes.
- **Test 2, aid off (189 s):** settled in 6.1 s; estimate held, controller error 0.07° rms (0.65° max during the planner
  switching below). Wheels up to 46 rad/s.
- **Truth:** no laser yet; the heading marks showed the heading held by eye. An independent magnetometer heading
  (tilt-compensated, wheel-corrected, gyro-free) moved −0.3…−0.45° over test 1 and ≤ 0.16° over the first 95 s of test 2
  (−0.5° after the pattern change): heading held within ~0.5°, limited by the field's speed dependence as the wheels
  filled. Too short to separate aid on from off; that needs long holds and the laser.
- **Bias:** while the table turned freely before test 1 (~100° of yaw) the aid moved the z bias −0.0379 → −0.0419 °/s
  (uncorrected hard iron). Test 2's flat start says the aid's value was close to right: the stand calibration was off.
- **Wheel-bias planner chatter (fixed, uploaded):** in test 2, when wheel B reached the 45 rad/s peak budget at 112 s,
  the planner changed pattern (by design, to cut the peak 45 → 30 rad/s) and then flipped between two patterns 15 times in
  6 s: all candidates had ~7.6 rad/s clearance (a near tie), and a wheel stuck near 0 by static friction during each change
  moved the decision. Fix (`null_bias.h`): decide on 0.5 s low-passed speeds, treat clearances within 1 rad/s as equal,
  and hold each change ≥ 5 s (uploaded 2026-10-31). Replaying test 2's wheel speeds through it: 1 change; motor-level simulation unchanged
  (hold 370 s vs 347 s without bias, 2 changes, 0.1° yaw error, fault test still passes).
- **Balance (free swings after each STOP, `tools/balance.py`):** swing period 2.87 s (10-21: 3.2–3.5 s), centre of mass
  6.1 mm below the bearing centre (was 4.5 mm), 0.24 mm −x / 0.12 mm −y off-centre, rest tilt +1.17° roll / −2.25° pitch
  (was −0.14° / −0.18°). The period is independent of the IMU mount, so the mass distribution changed since 10-21.
- **SD logger fix:** 0 rows lost over 9 min / 17 MB (earlier losses began 7–13 min in, so a longer log is still needed).
  IMU: 168 samples missed (0.3%), and one chip reset while the table was being floated: strain-relieve the STEMMA/INT/RST
  wiring.

### 2.18 Balance session and dead-motor false alarms (2026-10-31) — `test_runs/balance_iterations_2026-10-31.txt`, `DEMO_0004/0005.CSV`
- **Balance:** iterative trim with rest-tilt readings (`tools/rest_tilt.py`: BIAS 0 + DEMO LEVEL over the XBee). The IMU mount
  rocked (glued; `LEVEL ZERO` with a phone level, then re-done 1.5 h later: the glue crept 1.1° in pitch). Repeat readings
  scatter ~0.3°, plus ~0.3° that turns with heading (fixed in the room); readings at one heading (tape markers, wheel C at
  the tape) agree within ~0.15°. Then the loose heavy weights and top screws came off (swing 4.3 → 2.9 s, CG ~5.7 →
  ~13 mm below the bearing centre: twice as stiff, ~1,150 g·mm per degree, so a re-seated pack moves it only ~0.4°), and
  washers in the diagonal seats trimmed it (3 × 8 g on CD and on AD: +2.3° roll, as predicted). A wheel-A kick on the
  floating table swung it in roll only, confirming the IMU axes match the wheel layout. **Final rest tilt ≈ −0.9° roll /
  −0.7° pitch.**
- **Decision: stop fine balancing.** Requirements 1–3 run at the rest tilt (DEMO LEVEL), so they don't need it; tilted slews
  need the CG within ~0.02 mm, impossible by hand with packs re-velcroed at every charge (±1° per pack). Trim only if the
  rest tilt exceeds ~±5°.
- **Dead-motor false alarms (fixed, built, not uploaded):** 14 flags in the two logs (13 on D, 1 on A), every one a wheel
  starting from rest while the speed loop wound its duty up to 0.23–0.30 without the wheel moving (static friction). Healthy
  starts break away at 0.10–0.15 (up to 0.27). The detector now needs effort ≥ 0.4 (near the current limit) for a wheel at
  standstill; a replay of both logs gives 0 flags. Spinning-wheel detection (FAULT TEST) is unchanged. Uploaded; wheel D
  then tripped the encoder check instead (stuck > 0.5 s above 0.2 duty), so that threshold is now 0.4 too
  (uploaded). Wheel speed limit raised 60 → 80 rad/s (full motor torque up to ~79 rad/s at 12 V; uploaded).
- **Wheel D was weak, then wasn't:** 1 s at 0.3 duty reached 33–36 rad/s vs 46–47 for A–C (packs all 12.0 V). Later, after
  the pack-voltage check and a power cycle: encoder exact (10 hand turns = 4,002 counts), no drag by hand, and D 45.6–46.8 rad/s
  both directions (C 45.9–46.2). Likely an intermittent connection in D's power path (pack lead, driver terminals, motor
  leads) disturbed during the check: inspect it. Health check before runs: `TEST A..D 0.3` (table held), all ≈ 46 rad/s.
- **Vibration check at the new limit** (table clamped, `SPEED ALL` 0→760→−760 rpm, `STATIC_0008 31 Oct.CSV`): accel sd 0.02 m/s²
  at rest, ≤ 0.10 m/s² up to ±760 rpm, gyro unchanged; all wheels held 756–764 rpm at 0.47–0.48 duty. 80 rad/s is fine.
- **IMU sample loss is SD-logging stalls, not vibration:** pairs of 60–130 ms gaps (5–12 reports lost) every ~1.67 MB of log
  (~53 s), wheels stopped or spinning alike (same in DEMO_0004/0005: 0.3% lost, 140 ms at log open). Gaps over the 100 ms IMU
  timeout were skipped by the EKF (rotation lost); the timeout is now 300 ms so they are integrated (uploaded).
  Root cause in the SD path still open.

## 3. Open issues (details and owners in `TODO.md`)
- **Wheel D's encoder connection is intermittent** (reseated twice 2026-10-30): solder/crimp + strain relief.
- **Table balance / pendulum** (§2.7, §2.18): blocks levelling and tilted slews; requirements 1–3 work at the rest tilt.
- **Wheel D intermittent weakness** (§2.18): 25% slow earlier on 10-31, normal later; check its power-path connections.
- **Wheel A rub and low pack A; wheel C high duty and oscillation.**
- **Real table mass and inertia** still unmeasured (fits suggest x/y 0.05–0.13, z ~0.08 kg·m²).
- **Wheel-failure test** (requirement 3): all four wheels run again; ready for a first table run once wheel D's connection is permanent.
- BNO085 periodic 60–90 ms stalls (cause unknown, currently harmless).
- EKF noise retune from the static logs; wheel static-friction compensation; rotor momentum in the wheel model.

## 4. Firmware commands (quick reference; full list in `CLAUDE.md`)
`ARM`, `DISARM`/`KILL`, `STATUS`, `PING`, `CALIBRATE [s]`, `TEST <A-D> <duty>`, `SPEED <A-D|ALL> <rpm>`,
`COUNTS`, `ZERO`, `BEGIN STATIC TEST [rpm]`, `MAG MAP`, `DEMO SPIN`, `STOP`.

## 5. Data and analysis index (`test_runs/`)
| File | What it is |
|---|---|
| `2026-10-15_*` CSV/HTML | Bench speed sweeps, A/B driver swap, single-wheel and hand-turn tests (rpm 1.6× low) |
| `STATIC_0001.CSV`, `STATIC_0001_analysis.html` | First static IMU test: yaw drift, Allan variance, sample loss |
| `STATIC_0005.CSV`, `STATIC_motors_off_vs_on_analysis.html` | Motors-on static test, warm-up bias, EKF replay |
| `MAGMAP_0001.CSV`, `MAGMAP_0001_analysis.html` | Magnetometer offset vs wheel speed/direction |
| `MAGCAL_2026-10-27_t1..t4_*.txt` | Magnetometer calibration and yaw-aid tests (§2.12, report in `Reports/`) |
| `DEMO_0001.CSV` | First `DEMO SPIN` attempts on the air bearing (§2.7) |
| `STATIC_0008.CSV`, `STATIC_0009.CSV` | IMU sample-loss verification after the INT/RST/SD fixes (§2.8) |
