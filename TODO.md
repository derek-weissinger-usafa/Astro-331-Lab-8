# Lab 8 to-do list

Living list, newest priorities first. Last updated 2026-10-20 (IMU INT/RST fixes, STATIC_0009; DEMO_0001 findings). Earlier: 2026-10-15 static IMU test
(`test_runs/STATIC_0001_analysis.html`). Tick items off and add notes as you go.
Background on earlier findings: `HANDOFF_2026-10-15.md`. Commands and firmware details: `CLAUDE.md`.
**Everything done so far, for reports: `PROGRESS_LOG.md`.**

**Who:** 🔧 = hands-on / hardware (team) · 💻 = firmware or analysis (can be done with Claude) · 🧪 = test on the table

---

## 0. Magnetometer yaw aid (2026-10-27, fixed table) — IN PROGRESS, not yet within ±1°

Firmware (built and flashed): `mag_aid.h/.cpp` + `AttitudeEKF::updateMagYaw` (heading-only update, also
estimates the z gyro bias). Commands: `MAG ON`/`MAG OFF` (on by default), `KICK <deg>` (test: offset the yaw
estimate), `RAW ON/OFF` (100 Hz raw USB stream), `WHEEL <A-D> ON|OFF`. Encoder-fault detection now
disables a wheel automatically. Raw data: `test_runs/MAGCAL_2026-10-27_t1..t4_*.txt`.
- [x] 🔧 **Teensy power fault — RESOLVED 2026-10-30 (spare Teensy fitted).** After test 5 (2026-10-29) the
      Teensy stopped enumerating. Diagnosis: 5 V at its VIN pin but 0 V on 3.3V; 3.3V–GND shorted (0.8 Ω on the
      bare board, which ran very hot when powered) with every 3.3 V load removed, so the board itself failed.
      **The old Teensy is dead — do not power it again.** Cause not found: all four drivers' VIN–VCC and VM–VCC
      read open, and on every motor the red/black leads to encoder blue/yellow/white read open (12 V bridging onto
      an encoder was the top suspect; wheel D's encoder failed around the same time). Watch for a repeat.
      Spare Teensy 4.1 brought up in stages (flashed 5ec01a5 → VIN–VUSB trace cut attempted → VIN/GND only, 3.3 V OK →
      BNO085 + XBee → encoders → motors), checking 3.3 V and heat at each step; it stays cool.
      Results: XBee link OK; after reseating the IMU wiring, `rst=1`, `miss=0`, `maxgap=13` ms through
      calibration (a loose STEMMA/RST connection first gave 7 chip resets and ~1,100 lost samples). Hand-turn
      counts A −389 (1 turn), B −734 (2), C −1185 (3), clockwise = negative as expected, no swapped wires; D read
      0 (loose encoder wire, since fixed). `TEST x 0.3`: A +45.0, B +45.7, C +37.5 rad/s, so motor/encoder signs are right
      on the spare. IMU stayed clean with motors running. SD card moved to the spare (detected when a test opens
      a log). **Its VIN–VUSB pads are still joined** (the cut didn't separate them): upload over USB with the battery
      off, unplug USB, then battery on and work over the XBee (CLAUDE.md, "Uploading and running"). Wheel D: see below.
- [ ] 🔧 **Separate the spare Teensy's VIN–VUSB pads** (re-cut, confirm no continuity). Until then USB and the logic
      battery must never be on together, and there's no USB serial (no `RAW ON` / `tools/mag_cal.py`) while running.
      Report: `Reports/Teensy_Recovery_2026-10-30.html`.
- [ ] 🔧 **Wheel D encoder connection is INTERMITTENT (2026-10-30): make it permanent.** Cause of the old "one
      channel dead" symptom (counts ~0 while D spun, ±1 flicker, fake −165 rad/s spikes): an encoder wire almost pulled
      out. Reseated → `TEST D ±0.3` +44.9 / −46.8 rad/s (+2,445 / −2,606 counts). About 30 min later it failed again
      (no counts at 0.2–0.25 duty; the encoder-fault check disabled D). Reseated again → +44.2 / −45.3 rad/s,
      breakaway 0.10 duty like the others. **Solder or crimp the joint and add strain relief**, then wiggle-test it while
      polling `COUNTS`. If it drops out mid-run the firmware disables D, which would look like an uninjected wheel
      failure in FAULT TEST. D is in service (`WHEEL D OFF` not needed) while it works.
- [x] Findings: a stopped wheel's field depends on where its shaft stopped (1–5 µT ≈ 3–13°); a spinning wheel's
      mean field is repeatable and nearly speed-independent, but **reversing a wheel shifts it 0.6–1.6 µT**
      (modelled: `kDirStep`, `kSpeedSlope`, residual ≤ 0.8° per wheel alone). Horizontal field ≈ 21.5 µT
      (1 µT ≈ 2.7°).
- [x] Wheels stopped: works. A 10° yaw error decays with τ ≈ 25 s (−10° → −1.5° in 50 s).
- [x] 💻 Wheels spinning, gating flicker: fixed 2026-10-29 (low-passed speeds + 6/4 rad/s hysteresis; a
      field-strength reject no longer restarts the settle). **Needs the fixed-table re-test.**
- [x] 💻 Mixed-direction model error (1.5–2.9°): fixed 2026-10-29 with one spinning reference per wheel
      direction pattern. **Needs the fixed-table re-test.**
- [x] 💻 Reference inheriting a yaw error: references are now learned only within 30 s of an applied update
      (mode `x` = stale, gyro-only otherwise).
- [x] 💻 Manual `SPEED x 0` holding current: a stopped wheel at target 0 is now released (no drive, PI reset).
- [x] 🧪 Test 5 (2026-10-29, A+B only): spinning yaw within ±0.94° without kicks, kicks recover with
      τ ≈ 25 s. Report §1.6.
- [x] 🔧 **Wheel C slow / sticky start — resolved 2026-10-30.** It needed > 0.4 duty to start in test 5 and its first
      `TEST C 0.3` reached only 37.5 rad/s (A, B 45–46). Pack C reads 12.3 V like the others; retests gave +46.2 / −46.5 /
      +46.3 rad/s (A +46.0) and breakaway from rest at 0.08–0.10 duty, the same as A and B. Most likely sticky after
      sitting (grease / light rub). If a first start is slow again it may trip the dead-motor detector; give the wheels
      a short spin (`TEST x 0.3`) before a FAULT TEST run.
- [x] 🧪 **Wheel field recalibration, all four wheels (2026-10-30, `MAG MAP` → `test_runs/MAGMAP_0003.CSV`, analysis
      `test_runs/MAGMAP_0003_analysis.html`).** A–C direction steps repeat Owen's 2026-10-27 fit within ~0.15 µT; **D measured
      for the first time** (0.29, −0.99, 0.00 µT, the largest term); speed slopes ≤ 0.011 µT per rad/s; all four reversing
      together = sum of singles within 0.2–0.45°; stopped field varies ±7° with shaft positions. `kDirStep` / `kSpeedSlope`
      in `mag_aid.cpp` updated (uploaded 2026-10-30 with `MAG TEST`). (A first run, MAGMAP_0002, sat next to a steel tool cabinet:
      not used. Keep the table away from large steel.)
- [ ] 🧪💻 **Hard-iron calibration before any yaw motion.** The raw field changed since 2026-10-16 (|B| 50.5 → 64.0 µT, Earth
      ≈ 52 µT): there is now a large table-fixed offset. Harmless for heading hold (references learned in place) but it
      distorts the heading as soon as the table yaws (DEMO SPIN, slews). Fit it from a full yaw turn on the air bearing.
- [ ] 💻 **Wheel bias during heading holds (2026-10-30, built + simulated, uploaded 2026-10-31):** `ARM` / `DEMO LEVEL` keep
      every wheel ≥ 100 rpm from zero via the null mode (`null_bias.h`, `BIAS <rpm>`), so the aid isn't gated by stopped
      or slowly reversing wheels. Verify on the air bearing (`hb=` in STATUS, `mag_mode` in the log).
- [ ] 💻 **Predicted references (2026-10-30, built, uploaded 2026-10-31):** a new direction pattern the aid can't learn (stale)
      now copies the closest known pattern's reference instead of going gyro-only (`STATUS mpred=`). Check the accuracy
      on MAGTEST_0001 with `tools/replay` `--predict`.
- [x] 🧪 **MAG TEST, all four wheels (2026-10-30, `test_runs/MAGTEST_0001_report.html`): 36/37 PASS** on board; the miss
      was 1.02° at 300 rpm. Gyro alone: 5.7 °/h. Log replay: learning each new pattern at the current yaw random-walks the
      error; predicting it instead passes 37/37 (worst 0.77°). Prediction is now the firmware default (uploaded 2026-10-31).
- [x] 🧪 **MAG TEST rerun with prediction (2026-10-31, `test_runs/MAGTEST_0002_report.html`): stopped at 10/15** — the
      reversal model moved ~1° between sessions (raw |B| 71 → 65 µT). Replays of both runs: neither learning nor prediction
      is reliably within ±1° over 14+ pattern changes. Firmware back to learning in place (uploaded 2026-10-31).
- [x] 💻 **Uploaded 2026-10-31:** the learning default + SD logger fix (sector-aligned multi-sector writes, 96 KB buffer). Check
      `drop=` stays 0 over a long log.
- ~~Move the magnetometer away from the wheels~~ — **not possible (team decision, 2026-10-31)**: the IMU stays where it
      is, so the 0.3–1.2° per-new-pattern reference error is a fixed limit; keep pattern changes few during holds.
- [ ] 📊 **Update the final presentation for MAGTEST_0002 (deferred by the team, 2026-10-31).** Slides 4, 20 and 27 and the
      notes on slide 4 still say the current firmware replays 37/37 with predicted references, and `fig9_magtest.png` shows
      that as the gold "current firmware" line. Rework them to show both runs (run 1 36/37 learned; run 2 10/15 predicted;
      replays both ways) and the learning default. Builder: scratchpad `deck/build_deck.py` + `make_magtest_fig2.py`
      (rebuild from the team's current file, not an old copy).
- [x] 🧪 **DEMO LEVEL heading holds, aid on vs off (2026-10-31, `test_runs/DEMO_0003.CSV`, PROGRESS_LOG §2.17):** both held
      the heading (marks by eye; magnetometer heading within ~0.5°); aid in use 94%. Found and fixed wheel-bias planner chatter
      (15 pattern flips in 6 s at the peak budget) — **fix built, not uploaded.**
- [x] 💻 **Planner fix uploaded 2026-10-31.**
- [ ] 🔧 **Laser pointer** on the deck + wall marks (1° ≈ 5 cm at 3 m) for heading truth; then **long holds (≥ 5 min) aid on vs
      `MAG OFF`**, with `CALIBRATE 180` on the stand and the table not turned by hand before ARM (free turning moves the bias:
      hard iron uncorrected).
- [ ] 🔧 Strain-relieve the IMU STEMMA/INT/RST wiring (one chip reset while floating the table, 2026-10-31).

## 1. Next session: air-bearing run sheet (written 2026-10-30, in this order)

(The old §1 items from 2026-10-20 are done: IMU sample loss solved, STATIC_0009.)

- [ ] 🔧 Before: solder + strain-relieve wheel D's encoder lead; **weigh the table** (for `tools/balance.py`); have a
      spirit level; keep the table away from steel cabinets.
- [ ] 💻 **Upload** (USB with the battery off, CLAUDE.md "Uploading and running"). Check on USB: `GET` lists the
      tunables, `STATUS` ends with `hb=…,mpred=…`.
- [ ] 🧪 Battery on, **~10 min warm-up**, table clamped.
- [ ] 🧪 **`LEVEL ZERO`** with the deck level (spirit level): prints the IMU mounting roll/pitch and recalibrates. Note
      the values for `kImuMountRollDeg/PitchDeg`. Then **`CALIBRATE 180`**.
- [ ] 🧪 Release onto the air bearing. Nudge it and let it swing ~20 s with the wheels stopped, logging
      (`BEGIN STATIC TEST 0`) → `python tools/balance.py <log> --mass <kg>`.
- [ ] 🧪 **`STEP TEST`** (5° steps, ~2 min): note rise / overshoot / jitter and the measured **Izz** → `SET IZZ <value>`,
      repeat. If it overshoots or jitters, `SET ATT_ZETA` / `SET ATT_BW` and repeat (no upload needed).
- [ ] 🧪 **Requirement 1 + 2:** `DEMO LEVEL` ~5 min with the aid on (the wheel bias keeps it in use: `mag=S`, `hb=`);
      `STOP`. Then `MAG OFF`, the same again, `MAG ON`. Compare the yaw drift (SD logs; `tools/replay`).
- [ ] 🧪 **Hard iron:** `MAG OFF`, turn the floating table slowly through a full turn by hand (wheels idle), logging →
      `python tools/hardiron_fit.py <log>` → `SET MAG_HX/HY/HZ …`, `MAG ON`.
- [ ] 🧪 **Requirement 3:** `FAULT TEST B` then `DEMO LEVEL` (wheel B dies after 10 s; graded on board).
- [ ] 💻 Put the values that worked into `control_params.h` (Izz, gains, mounting, hard iron) and upload.

## 2. Hardware fixes

- [x] 🔧 **Wire BNO085 INT → Teensy pin 20 and RST → pin 21** (done 2026-10-20; firmware side done too).
- [ ] 🔧 **Wheel A rubs** (heard on the bench; needs ~0.42 duty vs 0.30 for B/D). Find the contact
      (wheel vs housing, wire, mount alignment).
- [ ] 🔧 **Charge pack A** (~1 V low). Charge all Epoch cells individually; each pack 12.4–12.6 V,
      cells within ~0.05 V.
- [ ] 🧪 **Wheel C: high duty and ±30–90 rpm oscillation, cause unknown** (gearing and encoder count
      are fine, no audible drag). Open-loop `TEST C 0.3` vs `TEST B 0.3` while measuring **VIN at driver
      C**; compare reported speeds. Suspects: pack C sagging under load, encoder at speed (3.3 V supply
      is below Pololu's 3.5 V min), weak motor.
- [ ] 🔧 Check the **TB9051FTG** datasheet: logic-high threshold at VCC = 5 V (Teensy drives 3.3 V;
      works in practice) and the max PWM frequency (firmware uses 20 kHz).
- [ ] 🔧 **Measure the real table** mass and, if possible, inertia (CAD or swing test). Firmware uses a
      5 kg hollow-cube model (I = 0.0623 kg·m²); the demo spin rate and controller gains depend on it.

## 2b. From the motors-on static test (STATIC_0005, `test_runs/STATIC_motors_off_vs_on_analysis.html`)

- [x] 💻 EKF bias settings tightened and z-bias locked to the calibration (replay: ~3 °/h after warm-up vs ~9 °/h old settings).
- [ ] 🧪 **Warm up before calibrating:** for yaw-sensitive runs, power on, wait ~10 min, then `CALIBRATE 180`. The gyro z bias moves ~25 °/h-worth in the first ~5 min.
- [ ] 🧪 **`MAG MAP`** on a FIXED table (~11 min, logs `MAGMAP_####.CSV`): maps the magnetometer offset vs each wheel's speed/direction. Then decide model vs. gating for a magnetometer yaw aid.
- [x] 💻 `DEMO SPIN` now holds a 100 rpm null-mode wheel bias (A,C +100 / B,D −100 rpm) so no shaft sits still (MAG MAP finding; `test_runs/MAGMAP_0001_analysis.html`). Rate budget now ~5.3 rpm.
- [ ] 🧪 Hard-iron (full-turn) magnetometer calibration from a `DEMO SPIN` log (wheels at the bias speed; skip the ~2 s windows where A/C cross zero).

## 3. Firmware (💻, mostly with Claude)

**Built 2026-10-30 evening, uploaded 2026-10-31 (not yet run on the table):**
- [ ] 💻🧪 **Runtime tuning** `SET <name> <value>` / `GET` (`tune.h`): attitude bandwidth/damping/Ki, inertias, speed-loop
      gains, mag heading noise, null gain, hold bias, mag hard iron, IMU mounting. Not saved: copy good values into
      `control_params.h`. First use: tune on the air bearing without re-uploading.
- [ ] 🧪 **`STEP TEST [deg]`** (`step_test.h`) on the air bearing: heading steps ±5° on top of DEMO LEVEL, graded rise /
      overshoot / settle / error / jitter, plus **I_zz from momentum exchange** (motor-level sim: within 3% of the
      expected value, drag torque recovered). Use the measured I_zz (`SET IZZ`) before tuning `ATT_BW`.
- [ ] 🧪 **`LEVEL ZERO`**: deck level by spirit level, table clamped → measures the IMU mounting tilt, rotates all IMU data
      into the table frame, recalibrates. Needed for requirement 2 (table attitude, not IMU-board attitude). Then put
      the printed roll/pitch into `kImuMountRollDeg/PitchDeg`.
- [ ] 🧪💻 **Hard-iron fit**: log a long yaw turn on the air bearing (`MAG OFF`, DEMO SPIN or by hand), run
      `tools/hardiron_fit.py <LOG>`, try the result with `SET MAG_HX/HY/HZ`, then copy into `kMagHardIronUt`.
- [ ] 🔧 **Balance for the slews (`tools/balance.py`, DEMO_0003_OLD):** the deck is level to ~0.03 mm, but the centre of mass
      is ~4.4 mm below the bearing centre (3.3 s swing). The ±10° slews need a ~55 s swing: raise the CG ~4.4 mm,
      ≈ 22,000 g·mm (e.g. 100 g raised 22 cm). Weigh the table (`--mass`) and re-run after each change (free swings
      after a DISARM are enough data).

- [x] **Interrupt-driven IMU reads** (INT pin 20, RST pin 21; `imu_bno085.cpp`). INT must NOT have the
      Teensy pull-up: with `INPUT_PULLUP` the line never read low and the chip sent nothing.
- [x] **SD stalls no longer starve the IMU** (IMU read in `yield()`, SD in DMA mode, skip writes while busy).
      Optional: preallocated file + `STOP LOG` command, only if `lat` grows again.
- [x] **IMU timeout** stays 100 ms: the remaining chip stalls are ≤ 90 ms, and the EKF now integrates
      gaps up to 100 ms instead of skipping anything over 50 ms.
- [ ] 🔧 **Trim roll balance (DEMO_0002, `PROGRESS_LOG.md` §2.10):** weights fixed pitch (≈ −0.3°) and cut
      gravity's pull ~3×, but roll still rests ≈ 3.3° (−y / wheel D side heavy → move weight toward +y /
      wheel B; ≈ 15 mN·m, e.g. ~50 g by ~3 cm; check with a bubble level first).
- [ ] 🧪 **`DEMO LEVEL`** (now holds the natural rest tilt, roll/pitch damped only, heading held): measure
      how long it holds before a wheel limit (expected minutes, limited by the ~0.12 mN·m yaw torque).
      This is the closest test to requirement 1 (heading hold).
- [ ] 💻 Optional: ramp the wheels down on a demo stop instead of braking (the brake kicks the table);
      `BALANCE` command; DEMO SPIN holding the natural tilt.
- [ ] **Retune EKF noise** from the static test: gyro noise 1e-3 → ~3e-4 rad/s/√Hz; accel σ 0.036 → ~0.01
      (measured ~0.002; leave room for wheel vibration).
- [ ] **Wheel speed loop:** add static-friction compensation (wheels don't move until ~27–29 rpm
      command, then jump); raise P gain / integral limit after the wheel A/C hardware is fixed.
- [ ] **Rotor momentum** in the wheel model (≈ N·J_rotor per wheel, a few %; needs 37D rotor inertia).
- [ ] Cosmetic: report roll/pitch σ in the level frame (the body-axis σ picks up yaw uncertainty
      when tilted, e.g. 0.88° reported vs ~0.02° actual).

## 4. Tests to come (🧪)

- [ ] **Static test with the motors spinning**: `BEGIN STATIC TEST` now holds all four wheels at 100 rpm (`BEGIN STATIC TEST 0` = motors off). Motor packs on, IMU on a still surface: magnetometer
      interference, vibration, accel noise → decides the gated-magnetometer yaw aid (motors-off heading
      was steady to ~0.7°, so it looks promising).
- [ ] **First `DEMO SPIN`** on the air bearing, after §1–§2 are done: motor packs on, table free to
      rotate (~1.5 turns), `STOP` / `KILL` ready on the XBee, watch the levelling phase. Expected
      ~6.4 rpm with the current model; logs to `DEMO_####.CSV`.
- [ ] Closed-loop `ARM` hold test (level, yaw 0), restrained first, then free.
- [ ] 🧪 **`SLEW TEST`** (stretch, first draft 2026-10-29): first run with yaw-only targets at the rest tilt
      (`SLEW SET n <rest roll> <rest pitch> <yaw>` for n = 1-4, rest tilt from `DEMO LEVEL`); simulated 4/4 PASS.
      The default ±10° tilts need near-neutral vertical balance (motor-level sim: a 60 s swing passes 4/4; today's
      3.3 s swing stops at the wheel limit 4.9 s in). Report: `Reports/Slew_and_Fault_Simulation.html`.
      Holds target 4 until `STOP`; tilt limit 25° while it runs (bubble has no top).
- [ ] 🧪 **Requirement 3: `FAULT TEST`** (first draft 2026-10-29, `wheel_fault.h/.cpp`): dead-motor detector
      (driven, not accelerating) + simulated failure + grading; dead wheel coasts, the other 3 carry on. Simulated
      9/9 PASS (detected 0.6-0.8 s, peak error ≤ 1.0°), no false alarms. **Needs all 4 wheels: fix wheel D's
      encoder first.** First run: `FAULT TEST A`, then `DEMO LEVEL`; then `FAULT TEST REAL` + pull a motor lead.
      On hardware check: the 800 rad/s² per unit effort figure (`TEST A 0.3` spin-up slope), and whether wheel C's
      stiction (> 0.4 duty to start) trips the detector. Optional: one driver EN/ENB line on a Teensy pin for a
      true coast (the firmware coasts at the zero-current duty instead).
- [ ] Stretch: 4-attitude optimal-path slew (firmware first draft: `SLEW TEST`, above).

## Notes

- Wheel-speed data in `test_runs/2026-10-15_*` (before the gear-ratio fix) reads **1.6× low**
  (motors are 6.3:1, firmware assumed 10:1). Multiply those rpm values by 1.6.
- `STATIC_0001.CSV` was logged with the 3 s calibration; its yaw drift (−27 °/h) is the "before" number.
- Accelerometer magnitude reads 0.981 g (1.8% low across all axes, not a tilt effect): small sensor
  offset/scale error, harmless because the filter only uses the direction.
