# Firmware review, 2026-11-02

A read-through of the whole PlatformIO project (`Astro 331 Lab 8/src`, `include`, ~6,200 lines) after the DEMO ALL run
of 11-01. **Nothing here is changed in the firmware: the code is frozen until after the final presentation.** Items are
suggestions, grouped by priority, each with the file and the reason. "Observed" means the evidence is in a log or
PROGRESS_LOG; "by inspection" means it comes from reading the code.

## P1: small, high value; do before the next hardware session

1. **Drive the PWM pins low before waiting for USB** (`main.cpp` `setup()`). Today `setup()` waits up to 3 s for a USB
   host *before* `WheelMotor::begin()` zeroes the PWM pins. On battery (no host) that is always the full 3 s with the
   Teensy's pins undriven. The TB9051FTG carriers may hold their inputs low with pull-downs, but we have not confirmed it,
   and the 16:48–17:07 wheel B runaway on 11-01 was most plausibly a boot with the motor switch on. Moving
   `analogWriteResolution` + `wheels[i].begin()` above the `while (!Serial …)` loop removes the question. One-line reorder.

2. **Report the reset cause at boot** (`main.cpp`). The Teensy rebooted on its own at least three times on 11-01
   (16:48, ~17:09, ~17:19) and we could not tell a brownout from a software reset. The i.MX RT1062's `SRC_SRSR` register
   says which: power-on / pin reset, watchdog, CPU lockup, software. Print it in the boot banner (and write it to the log
   header), then clear it. Ten lines; turns "unexplained reboot" into a diagnosis next time.

3. **Expose IMU queue drops in `STATUS`** (`imu_bno085.h/.cpp`, `main.cpp`). `Bno085Imu::queueDrops_` is counted but
   never reported. The queue is 64 samples ≈ 0.21 s of data at 3 reports × 100 Hz, so any `loop()` stall longer than
   that (the 1.2 s log-file creation on 11-01, the 0.3 s SD stalls) overflows it silently; the `miss=` counter then
   shows the loss indirectly. Add `qdrop=` to `STATUS` and raise `kQueueLen` to 256–512 (an `ImuSample` is ~24 bytes; the
   Teensy 4.1 has 1 MB of RAM). With the stall fixes of 11-01 this mostly matters for the log's continuity, but it is
   cheap insurance.

4. **Rate-limit the `# WHEEL LOOP STALLED` message** (`main.cpp` `wheelLoop`). It fired ~100 times in the 23:13 run, one
   per SD write. Each goes over the 9600-baud XBee (≈ 80 B of a 960 B/s link). Harmless so far, but it competes with
   data rows and command replies; print the first few and then a count every 10 s, and put the count in `STATUS`.

5. **Time out the DEMO ALL / FAULT TEST injection wait** (`wheel_fault.cpp` `FaultTest::step`, `main.cpp`). The failure is
   injected only once the target wheel turns ≥ 3 rad/s. During a hold the bias planner can park a wheel near zero for a
   long time, in which case part 3 waits silently. Add a timeout (say 30 s) that reports "wheel X never reached 3 rad/s"
   and skips or picks another wheel.

## P2: the SD stalls (root cause and fix)

6. **Preallocate the log file** (`data_logger.cpp`). The remaining 100–350 ms pauses come about once a second during a
   run (101 in the 23:13 run; 116 in the 22:35 run, so they predate the 11-01 changes). At ~26 KB/s the log crosses a
   32 KB cluster boundary about once a second, and every crossing makes SdFat read and write the FAT to allocate the next
   cluster, on a card whose programming time is tens to hundreds of ms. That is the same mechanism as the 1.2 s
   first-write stall at file creation, repeated per cluster. Fix: open the file through `SD.sdfs` as an `FsFile` and call
   `preAllocate(N)` (SdFat 2) with a generous size (e.g. 128 MB, enough for ~80 min) when the file is opened, before
   arming; then the writes never allocate. Truncate to the written length when logging ends (or leave it; the replay
   tools stop at the last row). Also lengthen `kSdFlushMs` from 1 s to ~5 s: each `flush()` rewrites the directory entry,
   another sector write per second; a power cut would then lose ≤ 5 s instead of ≤ 1 s, which is acceptable with the
   RAM buffer. Measure with the stall counter from item 4 before and after.

7. **Wheel loop on a timer interrupt** (`main.cpp`). Even with item 6, any SD or radio stall freezes the 100 Hz wheel loop
   because it runs from `loop()`. Running `wheelLoop()` from an `IntervalTimer` at 100 Hz makes the speed loops,
   current limit and fault detectors independent of the main loop. Design notes: the encoder counts are already
   interrupt-driven; `analogWrite` is safe from an ISR on the Teensy 4; share `wheelCmd[]`, `wheelEnabled[]` and the
   per-wheel mode through a small struct copied under `noInterrupts()`; queue the `# WHEEL FAULT` messages for `loop()`
   to print; keep `controlStep()` in `loop()` (it needs the EKF). This also removes the need for the 50 ms stall
   special-casing added on 11-01. It is the "right" fix but touches the most code; do it with the table clamped and the
   bench tests (`TEST`, `SPEED ALL`) first.

8. **Encoder-fault timer across stalls** (`main.cpp` `wheelLoop`). The 11-01 stall handling resets `encFaultS[i]` to 0 on a
   stalled step. With stalls about once a second and `kEncFaultTimeS` = 0.5 s, a genuinely stuck wheel is still caught
   between stalls, but only just. Better: hold the timer (neither accumulate nor reset) on a stalled step. Same for the
   dead-motor detector, which currently restarts its block (`reset()`): holding is enough. Both become moot with item 7.

## P3: behaviour on safety trips

9. **Ramp down instead of braking on a safety disarm** (`main.cpp` `disarm`, `wheel_motor.cpp`). Every disarm except a
   controlled DEMO STOP calls `WheelMotor::stop()`, which brakes (EN/ENB are hard-wired, so a zero duty is a brake).
   Braking dumps the wheels' net momentum into the table: with one wheel at 60 rad/s that is ≈ 0.013 N·m·s, a table
   spin of ≈ 10 °/s, exactly the "kick" seen on every STOP during the slew tests. For IMU_TIMEOUT, TILT, RATE and
   WHEEL_OVERSPEED a 2–3 s ramp to zero through the Manual-mode speed loop would leave the table still. Keep KILL as an
   immediate brake. (Hardware alternative: wire the drivers' EN pins to a Teensy pin so `coast()` is a true coast; today
   "coast" means driving at the back-EMF duty, which becomes a brake if the encoder is dead, as it did for wheel D.)

10. **IMU timeout while armed is 300 ms** (`control_params.h`). Reasonable, but note that after the queue fix in item 3 a
    long stall no longer loses samples, so the timeout only fires on genuine IMU silence; it could go back toward 150 ms
    once item 7 is in, to shorten the open-loop time if the IMU really stops.

## P4: estimator and control (no urgency; all verified working)

11. **EKF noise parameters are still placeholders** (`attitude_ekf.h`): `gyroNoise` 1e-3 vs ~3e-4 suggested by the static
    tests, `accelSigma` 0.036 vs ~0.01. They affect how fast the accelerometer corrects tilt and how much the P matrix
    builds the tilt/heading cross-terms that caused the heading leak (now projected out). Tune by replaying logs
    (`tools/replay --gyro-noise --accel-sigma`) against the laser runs; expect small gains in tilt noise, nothing in heading.

12. **Exact rotation vector in the controller** (`attitude_ctrl.cpp`): `thetaE_ = −2·vec(qe)` is the small-angle form
    (error ∝ θ³/24: 0.1% at 25°). `qToRotVec` is exact and already available. Cosmetic at our angles.

13. **Yaw gyro scale is applied before logging** (`main.cpp`). Logs since 10-31 carry z already scaled, and the replay tool
    needs to be told which scale (`--log-sz`). Log the raw z in an extra column, or write the scale into the log header,
    so a log is self-describing.

14. **`SET` values are lost at reboot** (`tune.cpp`). Several good values live only in `control_params.h` after a session
    (I_zz, gyro scale, hard iron). Saving the tunables to EEPROM (`EEPROM.put`) with a `SAVE` command, and loading them at
    boot with a version check, would avoid re-entering them and keep the firmware constants as the fallback.

15. **`main.cpp` is 1,860 lines.** The command handler, the calibration state machine, the test sequencers (MAG MAP, MAG
    TEST, STEP, SLEW, FAULT, DEMO ALL glue) and the logger glue could each be a module with a `start/step/stop` interface.
    No behaviour change; makes the next person's job easier. Do it after the presentation, with the build and the bench
    tests as the regression check.

## Things checked and found sound (so they need no change)
- `yield()` → `imu.background()` re-entry is guarded (`inLib_`), and the IMU timestamps are reconstructed from the INT
  edge; the 11-01 logs confirm sample timing against the chip's sequence counter (sum ratio 0.99974).
- The accelerometer gain projection (`accelNoHeading`) is applied after the z-bias row is zeroed and before the Joseph
  update, so P stays consistent; replay reproduces the flown heading within 0.015°.
- `allocate()` over any three wheels, the dead-wheel coast path and the `ALLOC` disarm with two wheels out behave as
  designed; the DEMO ALL hand-overs (hold → slew → fault) keep the controller state and wheel momentum continuous.
- The dead-motor detector's standstill threshold (0.4) and block logic give 0 false flags on 230 logged starts; detection
  on the table 0.49–0.91 s.
- Safety supervisor: KILL is parsed before any IMU work; tilt, rate and overspeed trips are checked every control step.

## Suggested order
1. Items 1–4 (an afternoon, no table needed; bench-test with `TEST` and `SPEED ALL`).
2. Item 6, then measure the stall count (needs the table only for a confidence run).
3. Item 7 as a separate branch, with full regression on the clamped table before a float.
4. Items 5, 8–15 as time allows.
