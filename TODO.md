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
- [ ] 🔧 **BLOCKER (2026-10-29): Teensy has no power / not detected.** After test 5 the board stopped
      enumerating on USB (no COM port, no bootloader device, no Windows plug-in event; tried after re-plugging).
      Team checks: 5 V present at the boost output, motor packs ≥ 12 V, wiring looks correct, but no LEDs anywhere,
      and all three checks failed (5 V at the Teensy VIN pin itself, 3.3 V at the Teensy 3.3V pin, red bootloader
      LED on pressing the program button). Next: with the logic switch on, measure VIN→GND **at the Teensy pin**
      (if 0 V, the boost→VIN lead/connector is open); if VIN is 5 V but 3.3V is ~0 V, unplug the 3.3 V loads one at
      a time (BNO085, XBee, the four encoder blue leads; wheel D's encoder first) to find a short; if VIN is 5 V and
      nothing is shorted, the Teensy itself may be dead (check for a hot regulator; swap in a spare Teensy 4.1 and
      re-flash). Also check the logic switch and the PKCELL voltage at the boost input under load.
      Note: the VIN–VUSB trace is cut, so USB alone does not power the Teensy.
- [ ] 🔧 **Wheel D encoder is broken** (one quadrature channel dead: counts stay ~0 while D spins at ~500 rpm,
      ±1 flicker and fake −165 rad/s spikes while still). Check D's yellow (pin 9) / white (pin 5) wires and the
      encoder board. Until fixed: `WHEEL D OFF` after every boot (the fault detector also trips it).
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
- [ ] 🔧 **Wheel C needed > 0.4 duty to start** in test 5 (fault detector disabled it). Check pack C voltage
      (and all packs: 12.4–12.6 V, cells within 0.05 V) before more motor tests.
- [ ] 🧪 Then: repeat test 5 with A, B, C (and D once repaired) (kick recovery stopped + spinning, 8 direction combos), then a ≥ 30 min drift
      run. Table hard-iron (rotates with the body) is not modelled; matters only for big yaw slews.

## 1. Next session: upload and diagnose (in this order)

- [ ] 💻 **Upload the latest `main`** to the Teensy (USB, COM3):
      `~/.platformio/penv/Scripts/pio.exe run -t upload --upload-port COM3` from the PlatformIO folder.
      It is NOT on the board yet. It adds: **60 s startup calibration**, `CALIBRATE`, IMU sequence
      diagnostics, `DEMO SPIN` / `STOP`.
      RST is wired now (2026-10-20): the firmware hardware-resets the BNO085 at startup, so uploads no
      longer need a power cycle. If the IMU isn't found, the firmware retries every second and prints
      `# ERROR: BNO085 not responding ... i2c: <addresses>` (`none` = no power/ground or I2C lines).
- [ ] 🧪 **Keep the table still for the 60 s calibration** after every boot (progress prints every 10 s
      over the XBee). If bumped, it restarts.
- [ ] 🧪 **Find where IMU samples are lost** (static test lost ~1/3 of gyro samples and froze up to 330 ms,
      38 times over 100 ms, in clusters ~20 s apart):
  - [ ] Idle ~5 min, **SD logging off** → send `STATUS`, note `miss=`, `chipgap=`, `maxgap=`.
  - [ ] Power-cycle, `BEGIN STATIC TEST`, ~5 min → `STATUS` again.
  - Reading it: large **`miss`** = reports the BNO085 sent but the Teensy never read. Large
    **`chipgap`** = the BNO085 itself sent late. `lat=` = worst INT-to-read delay (ms; datasheet wants
    ~1). If `miss`/`lat` only jump while logging → SD writes are the cause.
  - **2026-10-20 finding:** most of the old ~1/3 loss was the Adafruit library, not the bus: its
    `getSensorEvent()` keeps only the last report of each transfer, and gyro+accel+mag at 100 Hz often
    share one. Fixed (own SH2 callback + queue). First check, idle, SD off: `miss=0`, `maxgap=11` ms,
    `lat` < 0.05 ms. With SD on: `miss=102, lat=40` → yield() servicing → `miss=12, lat=43` → SD
    DMA mode → STATIC_0008 (`miss=370`, 314 from 2 chip resets) → INT edge-triggered reads + 5 ms
    fallback → **STATIC_0009: 25 of ~29,250 lost (0.085%), max gap 90 ms, no resets. Solved.** The rest
    are BNO085-internal 60–90 ms stalls (pairs ~22.4 s apart every ~132 s; not the calibration saves).
- [ ] 🧪 **Repeat the static test with the 60 s calibration** (~30 min, motors off, table still,
      `BEGIN STATIC TEST`). Expect yaw drift **~6 °/h** instead of 27 °/h. Drop the CSV in `test_runs/`
      for analysis; the new `gyro_seq` columns will show exactly which samples were lost.

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
- [ ] **Requirement 3: wheel-failure detection** and reconfiguration (allocation already supports
      3 wheels via `wheelEnabled[]`; detection not started).
- [ ] Stretch: 4-attitude optimal-path slew.

## Notes

- Wheel-speed data in `test_runs/2026-10-15_*` (before the gear-ratio fix) reads **1.6× low**
  (motors are 6.3:1, firmware assumed 10:1). Multiply those rpm values by 1.6.
- `STATIC_0001.CSV` was logged with the 3 s calibration; its yaw drift (−27 °/h) is the "before" number.
- Accelerometer magnitude reads 0.981 g (1.8% low across all axes, not a tilt effect): small sensor
  offset/scale error, harmless because the filter only uses the direction.
