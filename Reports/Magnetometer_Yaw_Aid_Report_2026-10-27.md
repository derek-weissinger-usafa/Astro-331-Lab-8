# Magnetometer calibration and yaw aid — work report, 2026-10-27

**Goal:** stop the yaw drift. Roll and pitch come from the accelerometer, but gravity carries no
heading, so yaw has been gyro-only and drifts at the residual z gyro bias (3–25 °/h depending on
warm-up; requirement 1: no visible drift, requirement 2: ±1° in all axes). The fix is to calibrate
the magnetometer against the reaction wheels' own fields and fuse it into the EKF for heading.

**Status: partly done.** The firmware is built and flashed. It holds heading well while the wheels
are stopped, but it is **not yet within ±1° while the wheels spin**. **Wheel D's encoder is broken**
and must be repaired before any 4-wheel air-bearing test.

All tests ran on the **fixed** (non-floating) table over USB (COM5), so the true yaw was 0 throughout.
Raw captures are in `test_runs/MAGCAL_2026-10-27_t1..t4_*.txt` (see "Data" at the end).

---

## 1. What was done

### 1.1 Firmware added (all in `Astro 331 Lab 8/`)

| Piece | File(s) | What it does |
|---|---|---|
| Raw USB stream | `src/main.cpp` (`printRawRow`) | `RAW ON` / `RAW OFF`: one row per magnetometer sample (~100 Hz) instead of the 20 Hz estimator CSV. Row: `R,t_us,mx,my,mz,countA..D,speedA..D (rad/s),dutyA..D,gyro_z,yaw_deg,state,mag_mode,mag_resid_deg`. Encoder counts give each motor shaft's angle (64 counts per motor turn). |
| Heading-only EKF update | `src/attitude_ekf.cpp` (`updateMagYaw`) | Compares the horizontal direction of the field (rotated to the inertial frame) with a reference. Corrects **yaw only** (the roll/pitch gain is projected out, so field errors can't tilt the estimate). It's the only update that moves the **z gyro bias**, which the accelerometer update keeps locked. Derivation in the code comment: H = [(A·ẑ)ᵀ, 0]. |
| Magnetometer aid | `include/mag_aid.h`, `src/mag_aid.cpp` | Removes the wheel fields, low-passes the field (τ = 0.5 s) and decides when the field may be used (see §1.3). |
| Aid tuning | `include/control_params.h` (`kMag*`) | Thresholds, settle/learn times, heading noise (σ = 0.3 rad per 100 Hz sample, which gives a ~25 s correction time constant). |
| Wheel fault detection | `src/main.cpp` (`wheelLoop`), `kEncFault*` | Disables a wheel whose encoder shows no motion while driven at \|duty\| > 0.2 for 0.5 s, or whose speed jumps faster than 3000 rad/s². A disabled wheel is never driven in any mode, and attitude control allocates over the rest. This is a start on requirement 3. |
| Commands | `src/command_link.cpp` | `MAG ON` / `MAG OFF` (aid on by default), `KICK <deg>` (test: offset the yaw **estimate** to watch the aid pull it back), `WHEEL <A-D> ON|OFF` (Idle only), `RAW ON|OFF`. `STATUS` adds `wheels=ABC-`, `mag=<mode>`, `mres=<deg>`, `bz=<deg/s>`. |
| Log columns | `src/data_logger.cpp` | SD/XBee rows end with `mag_mode,mag_resid_deg`. The USB estimator CSV has the same two columns appended. |
| Recalibration tool | `tools/mag_cal.py` | Runs the test 3 speed sweep for chosen wheels and prints `kDirStep` / `kSpeedSlope` rows for `mag_aid.cpp`. Plain Python (PlatformIO's Python has pyserial but no numpy). **Written but not yet run.** |

Mag-aid modes (`mag_mode` column / `STATUS mag=`): `-` off, `g` gated (or settling), `l` learning
the stopped reference, `L` learning a spinning reference, `s` in use (stopped), `S` in use (spinning),
`x` stale (needs a new reference but the yaw isn't fresh enough to learn one; added 2026-10-29).

### 1.2 Tests and findings

**Test 1: slow single-wheel spins** (±30 rpm, each wheel alone, ~5 min)
- Noise floor with everything stopped: 0.5–0.8 µT per sample.
- At slow speed the field swings 1.4–3 µT per sample. Only wheel B shows a clear pattern vs shaft angle
  (3rd and 6th motor harmonics, 2.7 and 1.9 µT: a 3-slot rotor). For A and C, most of the swing is
  fast noise (aliased PWM / commutation current), not shaft angle.
- The noise is mostly along x for A and C (azimuth 0°/180°) and along y for B, which matches where each
  motor sits.
- After `SPEED x 0` in Manual mode, the speed loop keeps about −0.06 duty on a stopped wheel (integrator
  hold). That puts current through a stopped motor and moves the field by µT.
- **Wheel D's encoder failed** (§1.4).

**Test 2: stopped field vs shaft angle** (short nudges + `DISARM`, 6 min)
- The 12 rpm nudges never broke static friction (max duty 0.04), so A, B and C didn't move. The test
  did show the stopped field is **stable to ~0.2 µT over 6 min** when nothing moves.
- D's encoder glitch looked like 36 rad/s to its speed loop, which drove D. D moved and the stopped
  field jumped 1.8 µT. This confirms that a wheel's stopping angle shifts the field by several µT.

**Test 3: steady speed sweep** (A, B, C alone at ±100/200/300 rpm, 12 s steps, ~6 min)
- Horizontal field here is about **21.5 µT, so 1 µT ≈ 2.7° of heading**.
- A spinning wheel's **mean** field is repeatable (revisiting +100 rpm agreed within 0.2 µT) and nearly
  independent of speed (≤ 0.3 µT from 100 to 300 rpm). 1-second means hold to 0.1–0.4 µT even though
  single samples spread 1.5–3 µT.
- **Reversing a wheel shifts the mean by 0.6–1.6 µT** (1.5–4°). That's the main term to model.
- Spinning mean vs stopped differs by up to 4.3 µT (wheel A). A reference must therefore belong to the
  wheel state it's used in.
- Model fitted per wheel and axis: `field = const + kDirStep·sign(ω) + kSpeedSlope·(|ω| − 10.47)`.
  Residual ≤ 0.15 µT rms, at most **0.3–0.8° of heading per wheel alone**:

  | Wheel | kDirStep x, y, z (µT) | kSpeedSlope x, y, z (µT per rad/s) |
  |---|---|---|
  | A | 0.319, 0.332, −0.082 | −0.0031, 0.0059, 0.0020 |
  | B | −0.471, 0.109, −0.007 | −0.0052, −0.0062, −0.0035 |
  | C | −0.717, −0.232, −0.005 | 0.0134, −0.0021, 0.0028 |
  | D | not measured (encoder dead), zeros | zeros |

**Test 4: aid running in closed loop** (~6 min)
- **Wheels stopped: works.** After `KICK +10`, yaw went from −10° to −3.2° at 30 s, −1.5° at 50 s and
  −0.65° at 70 s: a time constant of about 25 s.
- **Wheels spinning, 8 mixed-direction A/B/C combos (100–300 rpm): not good enough.**
  - The aid spent much of the time gated or re-settling (`L`/`g`) instead of applying updates.
  - In some combos the field heading was off from the reference by **1.4–2.9°**
    (e.g. A −200, B −300, C +100 rpm), which pulled yaw to −2°. Single-wheel residuals were ≤ 0.8°,
    so the wheel fields don't simply add.
  - `KICK −10` while spinning was never corrected (updates were gated). After `DISARM` the aid
    re-learned its stopped reference at the wrong yaw, so the 8.8° error stuck.

### 1.3 How the aid decides when to use the field (`mag_aid.h`)
1. Correct the raw field with the per-wheel spinning model (enabled wheels faster than 5 rad/s only),
   then low-pass it (τ = 0.5 s).
2. Classify the wheel state:
   - **Stopped:** every enabled wheel below 0.05 rad/s and its encoder count unchanged.
   - **Spinning:** every enabled wheel faster than 5 rad/s (~50 rpm).
   - Anything else is **gated**, and the gyro carries yaw through it.
3. Each state has its own reference field (inertial frame), learned by averaging A(q)ᵀm with the
   current attitude: 3 s for stopped, 10 s for spinning, after a 1.5 s / 2 s settle.
   - The stopped reference is captured right after calibration, which defines yaw 0. It's re-learned
     whenever the wheels stop at counts more than 1 count from the reference.
   - The spinning reference is learned the first time the wheels spin after a calibration.
4. Gate if the horizontal field strength is more than 15% off the reference, then call
   `ekf.updateMagYaw`.

Disabled wheels are assumed not to move, so their field becomes part of both references.

### 1.4 Wheel D encoder fault
- With D commanded to ±30 rpm, the count stayed at 0 while duty rose to the 0.34 current limit.
- The magnetometer showed a sharp 38 Hz line, which matches the 3rd motor harmonic of a ~500 rpm wheel
  folded down by the 100 Hz sampling. **D was spinning at ~500 rpm unseen.**
- On retest (`TEST D 0.2`) the team saw D move while `COUNTS` stayed at 0.
- While still, D's count flickered ±1 (98 changes in 6 min) and once spiked to −165 rad/s, which
  triggered a `WHEEL_OVERSPEED` disarm.
- Diagnosis: **one quadrature channel is dead.** With one channel stuck, the other only toggles the
  count up and down, so the net count is zero.

### 1.5 Follow-up, 2026-10-29: firmware fixes for the spinning case (built and flashed)

These changes address items 3–6 of §2. **Only the stopped case has been re-tested** (motors off:
`KICK +10` → −2.5° at 30 s, −1.1° at 60 s, −0.7° at 88 s, as before). The spinning case still needs
the test 4 re-run on a fixed table.
- **Hysteresis on the wheel state** (`kMagSpinEnterRadS` 6, `kMagSpinExitRadS` 4 rad/s, decided on
  speeds low-passed at 0.3 s). A sample rejected by the field-strength gate no longer restarts the
  settle; only a real change of wheel state or direction pattern does.
- **One spinning reference per direction pattern** (sign of each enabled wheel, up to 16). Mixed-direction
  combos no longer rely on single-wheel direction terms adding up, which is where the 1.4–2.9° came from.
  The measured `kDirStep` is kept but is constant within a pattern.
- **Learning guard:** a reference is learned only if a heading update was applied within
  `kMagLearnMaxGapS` = 30 s (or right after calibration). Otherwise the aid shows mode `x` (stale) and
  stays gyro-only until a state with a known reference returns.
- **Manual-mode release:** a wheel at target 0 whose ramp has finished and whose speed is below
  0.3 rad/s gets no drive, and its PI integrator is reset. No more ~−0.06 duty holding current in a
  stopped motor.
- Known limit: a `KICK` (artificial error) followed by a **new** direction pattern within 30 s is still
  learned into that pattern's reference. Real errors come only from gyro drift, which is ≤ 0.2° in 30 s.

### 1.6 Test 5, 2026-10-29: spinning validation of the fixes (`MAGCAL_2026-10-29_t5_spinning_validation.txt`)
Fixed table, D off. **Wheel C faulted 1.4 s into the first combo:** it needed > 0.4 duty to break away
(normal ≈ 0.08), so the encoder-fault detector disabled it just as it started to move. That suggests a
low pack C (battery check pending). **Only A and B were spinning**, so this validates the 2-wheel case.

| Phase | Yaw (true 0) | Aid in use | Field residual while used |
|---|---|---|---|
| Stopped, no kick | ±0.03° | 86% | 0.09° rms |
| Combos 1, 2, 5, 6 (no kick in or just before them) | within ±0.94° | 67–93% | 0.30–0.65° rms |
| `KICK` while spinning (combos 3, 7) | 10° → ~1° in 60 s (τ ≈ 25 s) | 67–93% | — |
| Repeated direction patterns (combos 7, 8) | stored references reused, no re-learn | 91–93% | — |
| Stopped after `DISARM` | −0.85° (inherited from the combo 7 kick) | 87% | 0.18° rms |

- The gating flicker is gone: the aid applied updates 67–93% of the time while spinning, against mostly
  `L`/`g` in test 4.
- The leftover ~0.9° traces to the artificial kicks: combo 4, combo 5 and the final stop learned their
  references while a kick's error was still decaying (the known limit above).
- `KICK +10` shows as −10° in the displayed Euler yaw (`perturbYaw` rotates about inertial +z, the
  opposite sense to the displayed yaw). It's cosmetic: recovery is unaffected.
- **Still to validate:** 3- and 4-wheel spinning (needs pack C checked and D's encoder repaired), and a
  ≥ 30 min drift run.

---

## 2. What still needs to be done

In order. 🔧 = hands-on hardware, 💻 = firmware/analysis, 🧪 = table test.

1. 🔧 **Repair wheel D's encoder.** Check the yellow (pin 9) and white (pin 5) wires at the Teensy and
   motor, the blue 3.3 V / green GND leads, and the encoder board. Verify by hand-turning 10 wheel turns
   with `ZERO` / `COUNTS` (expect ~4000). Until it's repaired, send `WHEEL D OFF` after every boot. The
   fault detector also trips it, but only after D has been driven.
2. 🧪 **Calibrate wheel D** once repaired: run `tools/mag_cal.py COM5 D` on the fixed table and paste the
   printed row into `kDirStep` / `kSpeedSlope` in `src/mag_aid.cpp`. Re-run A–C at the same time if
   anything near the IMU has moved.
3. 💻 **Stop the spinning-mode gating flicker.**
   - Decide the wheel state from low-passed speeds with hysteresis (e.g. enter Spinning above 6 rad/s,
     leave below 4).
   - Don't restart the 2 s settle on a single gated sample.
   - Wheel C's known ±30–90 rpm speed oscillation (TODO §2) makes this worse.
4. 💻🧪 **Fix the combined-wheel model error (1.4–2.9°).**
   - Extend the calibration to combinations of directions; the 8 direction patterns of A/B/C are enough
     for pairwise terms.
   - Or add a per-direction-pattern reference: learn one spinning reference per sign pattern. Learning
     inherits only the current yaw error, which is small right after an update.
   - Or gate updates for a few seconds after any wheel reverses.
5. 💻 **Protect the references from inheriting errors.** Re-learn a reference only if the aid applied
   updates within the last ~30 s (otherwise keep gating). If yaw σ is large, re-learn the stopped
   reference only from a spinning-mode fix.
6. 💻 **Stopped Manual wheels still carry current.** `SPEED x 0` holds ~−0.06 duty, which shifts the
   field. Either zero the integrator once the target is 0 and the wheel has stopped, or have the aid
   treat "stopped with current" as a separate state.
7. 🧪 **Re-validate on the fixed table** (repeat test 4):
   - `KICK ±10` recovery with wheels stopped and spinning.
   - The 8 direction combos, plus a ≥ 30 min drift run with the wheels moving.
   - Pass criterion: |yaw| ≤ 1° throughout, with no growing trend.
8. 🧪 **Air bearing:**
   - Power on, wait ~10 min for gyro warm-up, then `CALIBRATE 180`.
   - Then `ARM` / `DEMO LEVEL` with the aid on. Compare `yaw_deg` with `mag_resid_deg` and with a
     `MAG OFF` run.
9. 💻 **Table hard iron** (field fixed to the body) is not modelled. It doesn't matter for heading hold
   near the learned heading, but it does for big slews (stretch goal). Fit it from a full yaw turn on
   the air bearing (e.g. during `DEMO SPIN`): solve m − wheels = A(q)·B + h for B and h by least
   squares, using the gyro attitude over the turn.
10. 💻 **Retune `kMagHeadingSigmaRad`** (0.3 rad → ~25 s time constant) once the spinning case is clean.
    Faster pulls more model error into yaw; slower lets more gyro drift through.

### Useful procedures
- Calibration data capture: `RAW ON` over USB at 115200 baud. Each `R,` row is described in §1.1.
- The Teensy enumerates as **COM5** on this PC (older notes say COM3). Upload with
  `pio run -t upload --upload-port COM5`.
- Shell note: the Claude Code Bash tool collapses `\\n` in heredocs, so firmware edits made through
  inline Python scripts need care.

---

## 3. Data

| File (`test_runs/`) | Test |
|---|---|
| `MAGCAL_2026-10-27_t1_slow_single_wheels.txt` | Test 1: baseline, then ±30 rpm per wheel (D's encoder failure shows here) |
| `MAGCAL_2026-10-27_t2_stopped_nudges.txt` | Test 2: stopped-field stability, D glitch-driven move |
| `MAGCAL_2026-10-27_t3_speed_sweep.txt` | Test 3: A/B/C speed sweeps (model fit source) |
| `MAGCAL_2026-10-27_t4_aid_validation.txt` | Test 4: closed-loop aid, kicks and 8 direction combos |

Format: one line per received serial line, `<seconds since start> <line>`. Data rows start with `R,`
(columns in §1.1). Test 4 rows carry the `mag_mode` and `mag_resid_deg` columns; tests 1–3 predate them.
Commands sent appear in the console logs as `>> CMD`. Schedules: test 1 starts each wheel at
30/90/150/210 s; test 3 starts wheel A at 8 s, B at 116 s, C at 224 s with 12 s steps.
