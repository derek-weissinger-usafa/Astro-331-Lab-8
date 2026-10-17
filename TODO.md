# Lab 8 to-do list

Living list, newest priorities first. Last updated 2026-10-15 after the static IMU test
(`test_runs/STATIC_0001_analysis.html`). Tick items off and add notes as you go.
Background on earlier findings: `HANDOFF_2026-10-15.md`. Commands and firmware details: `CLAUDE.md`.

**Who:** 🔧 = hands-on / hardware (team) · 💻 = firmware or analysis (can be done with Claude) · 🧪 = test on the table

---

## 1. Next session: upload and diagnose (in this order)

- [ ] 💻 **Upload the latest `main`** to the Teensy (USB, COM3):
      `~/.platformio/penv/Scripts/pio.exe run -t upload --upload-port COM3` from the PlatformIO folder.
      It is NOT on the board yet. It adds: **60 s startup calibration**, `CALIBRATE`, IMU sequence
      diagnostics, `DEMO SPIN` / `STOP`.
      If the board sends nothing after an upload, **power-cycle** (logic off, unplug USB, wait, replug):
      the BNO085 can hang because its RST pin isn't wired.
- [ ] 🧪 **Keep the table still for the 60 s calibration** after every boot (progress prints every 10 s
      over the XBee). If bumped, it restarts.
- [ ] 🧪 **Find where IMU samples are lost** (static test lost ~1/3 of gyro samples and froze up to 330 ms,
      38 times over 100 ms, in clusters ~20 s apart):
  - [ ] Idle ~5 min, **SD logging off** → send `STATUS`, note `miss=`, `chipgap=`, `maxgap=`.
  - [ ] Power-cycle, `BEGIN STATIC TEST`, ~5 min → `STATUS` again.
  - Reading it: large **`miss`** = reports the BNO085 sent but the Teensy never read (our side: no INT
    wiring and/or SD writes blocking the loop). Large **`chipgap`** = the BNO085 itself sent late.
    If `miss` only jumps while logging → SD writes are the cause.
- [ ] 🧪 **Repeat the static test with the 60 s calibration** (~30 min, motors off, table still,
      `BEGIN STATIC TEST`). Expect yaw drift **~6 °/h** instead of 27 °/h. Drop the CSV in `test_runs/`
      for analysis; the new `gyro_seq` columns will show exactly which samples were lost.

## 2. Hardware fixes

- [ ] 🔧 **Wire BNO085 INT → Teensy pin 20 and RST → pin 21** (two jumpers; STEMMA QT only carries
      power + I²C). INT: the datasheet wants the host to answer data-ready within ~1 ms at 100 Hz or
      the chip starves and drops outputs; we currently don't watch it at all. RST: lets firmware
      reset the IMU, fixing the post-upload hang. Then tell Claude to add the firmware side (§3).
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

- [ ] **Interrupt-driven IMU reads** once INT is wired (pin 20), plus RST in `pins.h` (pin 21).
- [ ] **Non-blocking SD logging** if gaps persist while logging: preallocated contiguous file + only
      write when the card isn't busy (Teensy SDIO "logger" pattern), plus a `STOP LOG` command to close
      the file cleanly (otherwise a power cut leaves empty space at the end, trimmed in analysis).
- [ ] Decide the **IMU timeout** (100 ms now) after the diagnosis. Gaps > 100 ms disarm the wheels
      when armed, which would cut a `DEMO SPIN` short.
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
- [ ] **Requirement 3: wheel-failure detection** and reconfiguration (allocation already supports
      3 wheels via `wheelEnabled[]`; detection not started).
- [ ] Stretch: 4-attitude optimal-path slew.

## Notes

- Wheel-speed data in `test_runs/2026-10-15_*` (before the gear-ratio fix) reads **1.6× low**
  (motors are 6.3:1, firmware assumed 10:1). Multiply those rpm values by 1.6.
- `STATIC_0001.CSV` was logged with the 3 s calibration; its yaw drift (−27 °/h) is the "before" number.
- Accelerometer magnitude reads 0.981 g (1.8% low across all axes, not a tilt effect): small sensor
  offset/scale error, harmless because the filter only uses the direction.
