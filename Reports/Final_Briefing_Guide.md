# Briefing guide: Final Presentation

For presenting the 19 slides of `Astro 331 Lab 8_2_3 Body Final Pres.pptx` (slides 20 on are backup, not presented).
Target 20–30 minutes: about 1–1.5 minutes per slide, a little more on the DEMO ALL slides.
Each section has a short script, an explanation of every item on the slide, and likely instructor questions with
answers. Code references are in the PlatformIO project (`Astro 331 Lab 8/`); the full record is `PROGRESS_LOG.md`.
The prelim's `Theory_Briefing_Guide.md` still holds the deeper derivations for slides 4 and 5.

---

## The story in one paragraph (know this cold)
> "We built a 3-axis reaction-wheel attitude control system on the air bearing table: our own Kalman filter fuses the
> gyro and accelerometer (and, where the field allows, the magnetometer), a quaternion controller computes a torque,
> and four wheels in a pyramid produce it, with any three enough to keep control. We verified every requirement on the
> floating table in a single command, DEMO ALL, against an independent truth: a laser on the table read on a wall
> 2.48 m away. Getting there, the laser exposed two problems the firmware could not see in itself: the estimator was
> losing about 1% of every turn, and every first run after a boot failed because the SD card froze the wheel loop.
> Both are fixed, and the final run passed: heading held to 0.04°, back within 0.05° after a full turn, and a wheel
> failure caught in half a second."

## Numbers to have ready
| What | Value | Where it comes from |
|---|---|---|
| Heading hold, 60 s (DEMO ALL) | max 0.04°, jitter 0.011° rms, drift 0.000° | `DEMO_0020`, graded on board |
| Heading hold, 5 min (laser) | within a 0.25° band, no trend | 11-01 18:34 run |
| Heading after a full 360° of slews (laser) | ≈ 0.05° (was +2.3° CCW / −1.8° CW before the fix) | DEMO ALL; mirrored test |
| Slew targets | 4/4, max yaw error 0.03 / 0.02 / 0.13 / 0.05° | DEMO ALL part 2 |
| Wheel failure | detected 0.50 s, back within ±1° in 0.5 s, peak 0.90° (laser 0.3°), 30 s hold ≤ 0.36° | DEMO ALL part 3 |
| Detection on the table (4 tests) | 0.49–0.91 s | FAULT TEST 10-31, DEMO ALL runs |
| Roll/pitch noise at rest | σ 0.02° | static test 10-15 |
| Laser geometry | R = 2.48 m; 0.1° = 4.3 mm; reading ≈ 0.05° | `tools/laser_scale.py` |
| Gyro yaw drift after warm-up | ≈ 3 °/h (replay); laser holds show no trend over 5 min | static test, laser holds |
| Early calibration (5 min after power-on) | ≈ 15 °/h drift (−0.78° in 2.5 min) | laser hold 11-01 |
| Table inertia | I_zz = 0.077 kg·m² (STEP TEST); I_xx, I_yy ≈ 0.15 (swing fits) | slides 5, backup |
| Wheel inertia | J = 3.06×10⁻⁴ kg·m² (CAD) | slide 5 |
| Wheel speed limit | 80 rad/s command, run stops at 75 rad/s, disarm at 90 | `control_params.h` |
| Momentum capacity | ≈ 32 mN·m·s per tilt axis | 2·cos45°·J·75 rad/s |
| Controller | ωₙ = 2 rad/s, ζ = 0.9, no integral term | slide 5 |
| Rates | IMU, estimator, controller and wheel loops all 100 Hz | |
| CG below the bearing (demo setup) | ≈ 9–11 mm (swing ≈ 3.2–3.6 s); rest tilt −2.5 / +0.1° | balance 11-01 |

---

## Slide 1: Title
Introduce the team and the project name. Optional: "Our goal was a satellite-style attitude control system you can test
on the ground. Here is how we built it, how we verified it, and what we learned."

---

## Slide 2: Why Attitude Control Matters

### The 30-second version
> "Everything a satellite does depends on pointing. Antennas, solar arrays and cameras only work when aimed, so we have
> to point. We can only point what we can measure, so we have to know our attitude. And reaction wheels are the parts
> that wear out: Kepler lost two of four and its primary mission ended, so a wheel failure must not end ours. Those
> three jobs are our three requirements. The air bearing lets us test the whole loop on the ground."

### Item by item
- **POINT → Req. 1, hold a heading.** Reaction wheels give fine, continuous torque with no propellant: ideal for holding
  and slewing.
- **KNOW → Req. 2, attitude to ±1°.** Sensor fusion turns three imperfect sensors into one estimate: the gyro is smooth
  but drifts, the accelerometer knows "down" but not heading, the magnetometer knows heading but is disturbed easily.
- **SURVIVE → Req. 3, detect and compensate.** Kepler (2013) and Hubble's gyros are the classic cautionary tales.
  Four wheels in a pyramid mean any three still control all three axes.
- **Objective box:** 3-DOF reaction-wheel attitude control on the air bearing table, with gyro, accelerometer and
  magnetometer fusion. **Why an air bearing:** the table floats on a thin film of air on a spherical cup, so it can
  rotate in roll, pitch and yaw with almost no friction.

### Likely questions
| Question | Answer |
|---|---|
| How is the air bearing like orbit? | Nearly frictionless rotation in three axes, like a free body. Two differences: gravity (the table is a pendulum if the centre of mass is below the pivot) and a small air-flow torque in yaw. Both show up in our data. |
| What happened to Kepler? | Two of its four reaction wheels failed (2012–2013); with only two it could not hold its fine pointing, so the primary mission ended and it was repurposed as K2 using solar pressure as a third "wheel". |

---

## Slide 3: Requirements: Verification & Status

### The 30-second version
> "All three threshold requirements are verified on the floating table, all in one run and all against the laser.
> Heading held to four hundredths of a degree for a minute; after a full turn of slews we came back within five
> hundredths; and when we cut a wheel's motor the firmware caught it in half a second and held attitude on three wheels.
> The stretch goal is met for yaw targets; tilted targets need a better-balanced table."

### Row by row
- **(1) Hold a heading.** Pass criterion: free table, commanded heading, 1 min, |error| < 1°, no visible jitter.
  Evidence: DEMO ALL part 1 (max 0.04°, jitter 0.01° rms, laser steady) and a 5-minute laser hold within 0.25°.
- **(2) Attitude to ±1° in all 3 axes.** Evidence: after 360° of slews the laser read 0.05° from where it started
  (heading); roll and pitch from the accelerometer, σ 0.02° at rest.
- **(3) Detect a wheel failure and compensate.** Evidence: wheel B's motor cut without telling the controller; detected
  0.50 s; back within ±1° in 0.5 s; held ≤ 0.36° for 30 s on wheels A, C and D.
- **Stretch: 4-attitude optimal slew.** SLEW YAW 4/4 (shortest path between headings is a turn about the vertical).
  The half-circle: tilted targets need the centre of mass within ~1 mm of the pivot (slide 10).
- **Symbols:** ✔ verified on hardware, ◐ built/simulated or met in part. This was prelim feedback (enlarge, use checkmarks).

### Likely questions
| Question | Answer |
|---|---|
| Who decided pass/fail? | The Teensy grades each part against the thresholds in the firmware (`full_demo.cpp`, `wheel_fault.cpp`, `slew_test.cpp`), and the laser is the independent check. They agree. |
| "All three axes" but your truth is heading only? | Yes: the laser measures heading. Roll and pitch come from the accelerometer, which measures gravity directly (0.02° noise at rest, mount offset calibrated with a level). Independent 3-axis truth is in our future work (AprilTags). |
| Why is the stretch only half-filled? | Tilting a pendulous table costs the wheels steady torque; at our balance a held tilt fills a wheel in seconds. Yaw-only targets meet the requirement's "shortest path" between the four attitudes we could hold. |

---

## Slide 4: Theory: Estimation & Control Loop

### The 30-second version
> "This is the whole loop. Input: the commanded attitude and rate from whichever mode is running. The IMU gives a gyro
> sample every 10 ms; our estimator turns gyro and accelerometer into attitude; the controller compares it with the
> reference and computes a torque; allocation splits the torque over the wheels; each wheel has its own speed loop.
> Output: the table's attitude. Gravity and air drag are the disturbances. One thing changed since the prelim: the
> accelerometer may now only correct roll and pitch, never heading."

### Box by box
- **INPUT (green):** q_ref, ω_ref, α_ref from the active mode (ARM, DEMO LEVEL, STEP / SLEW TEST, DEMO ALL).
- **BNO085 IMU (yellow):** raw gyro, accelerometer and magnetometer at 100 Hz each; interrupt-driven reads so timing is
  exact. We do our own fusion rather than the chip's black-box output.
- **MEKF + mag aid:** multiplicative EKF, 6 error states (3 attitude + 3 gyro bias). Gyro predicts; accelerometer corrects
  roll/pitch; the magnetometer aid can correct heading, but is off at the demo site (slide 9).
- **Attitude PD → Allocation → Wheel PI ×4:** slide 5.
- **Fault detector:** watches each wheel's drive and response; a dead motor drops the allocation to three wheels (slide 7).
- **Table + 4 wheels:** I ω̇ = −ḣ_w − ω × (Iω + h_w) + τ_dist. The table turns opposite to the wheels' momentum change.
- **OUTPUT (red):** q, ω. Disturbances: gravity from the CG offset (pendulum), yaw drag from the air bearing.
- **Telemetry:** every sample to the SD card (100 Hz), every 50th over the XBee radio (2 Hz).

### Likely questions
| Question | Answer |
|---|---|
| Why your own filter instead of the BNO085's fusion? | We can see and tune every step, track uncertainty, and control how the magnetometer is used. The heading-leak bug (slide 12) could only be found and fixed because we own the filter. |
| What does "multiplicative" mean? | The quaternion is kept separately; the filter estimates only a small 3-angle error on top and folds it in by rotation, so the quaternion stays a valid rotation. |
| Why 100 Hz? | Well above the control bandwidth (2 rad/s ≈ 0.3 Hz) and within the IMU's I²C budget; one gyro sample drives one estimator + control step. |
| What happens if the IMU data stops? | The supervisor disarms after 300 ms without a gyro sample; shorter gaps are integrated with the next sample's rate. Other trips: tilt > 30°, rate > 2 rad/s, wheel > 90 rad/s, KILL. |

---

## Slide 5: Theory: Control Law & Wheel Allocation

### The 30-second version
> "The controller is a PD law written for quaternions, with feed-forward so it can follow a moving reference. Its
> output torque goes to four wheels in a 45° pyramid: any three still span all three axes, which is the basis for
> requirement 3. The pseudo-inverse picks the smallest wheel torques that make the requested torque. And the physics
> limit: wheels can only move momentum around, never get rid of it, so any steady disturbance eventually fills them."

### Term by term
`τ = −Kp θₑ − Kd (ω − ω_ref) + I α_ref + ω × (Iω + h_w)` (`src/attitude_ctrl.cpp`)
- **−Kp θₑ:** spring; θₑ from the error quaternion, shortest way round, any orientation.
- **−Kd (ω − ω_ref):** damper on the rate error (uses the gyro directly, no noisy differentiation).
- **+I α_ref:** feed-forward: the torque a moving reference needs is known in advance.
- **+ω × (Iω + h_w):** cancels gyroscopic cross-coupling, so each axis behaves independently.
- **Kp = I ωₙ², Kd = 2ζ I ωₙ:** pole placement on I θ̈ = τ. ωₙ = 2 rad/s, ζ = 0.9: ≈ 1.5 s rise, no overshoot.
  Yaw (I_zz 0.077): Kp ≈ 0.31 N·m/rad (5.4 mN·m per degree), Kd ≈ 0.28 N·m·s/rad. No integral (wheels already
  integrate a steady torque).
- **Pyramid:** each wheel axis 45° above horizontal, pointing to +x (A), +y (B), −x (C), −y (D). A and C share roll,
  B and D share pitch, all four share yaw.
- **u = −D⁺τ over the enabled wheels:** D's columns are the wheel axes; the pseudo-inverse is the least-total-torque
  solution; drop a disabled wheel's column and the same formula runs on three.
- **Null mode [+1, −1, +1, −1]:** zero net torque; holds every wheel ≥ 100 rpm from zero during holds.
- **Inner loop:** 100 Hz speed PI per wheel with a back-EMF current limit (2.5 A of a ~5.5 A stall; no current sensor).
- **Numbers box:** wheel J 3.06×10⁻⁴ kg·m² (CAD); table I measured (I_zz 0.077 by STEP TEST, I_xx, I_yy ≈ 0.15 by
  swing fits); speed limit raised 60 → 80 rad/s ≈ 32 mN·m·s per tilt axis.

### Likely questions
| Question | Answer |
|---|---|
| The roll/pitch gains use 0.062 but the table is 0.15? | In the demo modes roll and pitch are damping-only (the tilt reference follows the measured tilt), so only Kd matters there and it is well damped. Yaw uses the measured 0.077. |
| Why no integral term? | A constant disturbance would just fill the wheels faster; the wheels already integrate. The fix for steady torque is balance or momentum dumping. |
| How long can you hold? | Capacity ÷ disturbance. Yaw drag (~0.1 mN·m) fills the wheels in ~5 minutes; a held tilt on our pendulous table costs mN·m per degree and fills them in seconds. |
| Why 80 rad/s? | The motors give full torque up to ~79 rad/s on 12 V packs; vibration stays small up to 760 rpm. The run stops at 75, disarm at 90. |
| Why a pyramid? | Three orthogonal wheels lose an axis when one fails; four in a pyramid keep all three axes with any three. |

---

## Slide 6: Code: Gravity Update Fix

### The 30-second version
> "This is the most important bug we found. Gravity tells the filter which way is down, never which way it is facing.
> But while the table turns, the filter's heading error becomes correlated with its tilt error, and the standard
> Kalman gain then lets each accelerometer update nudge the heading too. It took about one percent off every turn. The
> fix is one projection: remove the part of the gain that points along vertical. After it, a full turn comes back within
> 0.05° by laser."

### Item by item
- **Code, top half (`attitude_ekf.cpp`):** measured "up" y vs predicted "up" ŷ = A·ẑ; residual r = y − ŷ;
  H = [−[ŷ]×, 0]; Kalman gain K; z-bias row zeroed (locked at calibration).
- **The new loop:** for each column of K, subtract its component along ŷ. A correction about the vertical is a heading
  change, so after the projection the accelerometer can only tilt the estimate.
- **Why it leaked:** the prediction step's transition matrix contains −[ω]×: while turning at slew rates, tilt and
  heading errors rotate into each other, so the covariance P builds cross-terms between them. The optimal gain then
  "uses" gravity to correct heading through those cross-terms; with any model mismatch that is a systematic pull.
- **Bottom line of the code:** the yaw gyro scale, refitted from laser runs: 1.0045 (was 1.009, which came from a
  magnetometer comparison with its own ~0.5% uncertainty).
- **Right column:** symptom → clue → cause → fix → result. Stat cards: +2.3° / −1.8° before, 0.05° after.

### Likely questions
| Question | Answer |
|---|---|
| Isn't the full Kalman gain optimal? | Only if the model is perfect. Gravity physically contains no heading information, so any heading correction from it comes from cross-correlations built on model assumptions (noise values, rates). Removing it is the standard "decoupled" accelerometer update in attitude filters. |
| How did you find it? | The laser disagreed with the estimator after full turns, and the error flipped sign when we turned the other way. Integrating the logged gyro gave 363–364° where the estimator said 360°: the gyro was right, the filter was removing rotation. |
| How did you choose 1.0045? | Replayed the logged runs on the PC through the fixed estimator (`tools/replay`, the firmware's own code) and fitted the scale that matches the laser in both directions; residual ±0.1° plus a common ~0.3° bias offset. |
| Why didn't simulation catch it? | The simulation used a perfect estimate for control; the estimator was checked separately in 3-axis motion but not over full-turn slews against a long reference. Lesson: verify against independent truth on the hardware. |

---

## Slide 7: Code: Wheel-Failure Detection

### The 30-second version
> "We have no current sensor, so we infer motor current from the duty cycle and the wheel speed. A healthy motor that is
> pushed hard must speed up; a dead one doesn't. Every tenth of a second we compare the speed gained with what the push
> should have produced. Two bad blocks in a row and the wheel is taken out of the allocation and allowed to coast; the
> other three carry on. On the table it detects in 0.49 to 0.91 seconds."

### Item by item
- **effort = duty − speed/160:** a DC motor's current is proportional to applied voltage minus back-EMF; duty ×
  pack voltage is the applied voltage and speed/160 is the back-EMF fraction (160 rad/s per unit duty, measured).
- **Driven hard:** mean |effort| ≥ 0.2 over a 0.1 s block (0.4 for a wheel at rest: static friction needs up to 0.27).
- **predicted = 800 × ∫effort dt:** expected speed gain (stall torque ÷ wheel inertia, derated).
- **bad = gained < 25% of predicted;** 0.2 s of bad blocks = dead.
- **Response:** wheelEnabled = false (allocation over three), coast (zero current), not brake: braking would dump the
  wheel's momentum into the table.
- **FAULT TEST:** `setSimulatedDead` holds the motor at zero current while the firmware still believes it is driving it:
  the detector must find it on its own. Graded on board: detect ≤ 1 s, back within ±1° ≤ 10 s, then ±1° for 30 s.
- **Stat cards:** 0.49–0.91 s on the air bearing (4 tests); 0 false alarms in 4 healthy simulated runs.

### Likely questions
| Question | Answer |
|---|---|
| Is a simulated failure realistic? | The motor really gets zero current (the PWM is changed under the controller); only the bookkeeping knows. We also had a real one: wheel D's encoder lead opened in a run and the detector took D out (backup). |
| Could a healthy wheel be flagged? | Early on, yes: wheels stuck by static friction at rest. We raised the threshold at standstill to 0.4; replaying 230 starts gives 0 false flags. We also skip the check across loop stalls (slide 13). |
| Why not watch the speed error? | A speed error also appears when the controller asks for big changes or the wheel is near its limit; comparing effort with response is specific to "pushing but nothing happens". |
| What if two wheels fail? | Two wheels cannot span three axes; the allocation fails and the supervisor disarms (`ALLOC`). |

---

## Slide 8: Hardware: Laser Heading Truth

### The 30-second version
> "The estimator can't grade itself: it uses the same gyro it is trying to check. So we mounted an 11-gram laser on the
> deck and printed a scale for a wall 2.48 metres from the bearing. A tenth of a degree moves the dot 4.3 millimetres,
> easy to read by eye. Every heading number from here on is checked against this."

### Item by item
- **Geometry:** x = R tan θ, R = 2480 mm. 0.1° = 4.3 mm, 0.5° = 21.6 mm, 1° = 43.3 mm.
- **Scale:** printed by `tools/laser_scale.py`: 0.5° ticks to ±5°, 0.1° ticks within ±1°, positive = counter-clockwise
  (same sign as the firmware's yaw), with a 100 mm check bar to confirm the print scale.
- **Use:** line the dot up on 0 before a run; read at each checkpoint (end of the hold, end of the slews, through the
  failure).
- **Limits:** heading only; reading resolution ≈ 2 mm ≈ 0.05°; the laser is 11 g, so it barely shifts the balance.

### Likely questions
| Question | Answer |
|---|---|
| How accurate is it? | ≈ ±0.05° reading; the distance error is small (1% of R is 1% of the angle, 0.0005° at 0.05°). Independent of the IMU and the firmware. |
| Doesn't the table translate? | The bearing is a sphere: the table rotates about a fixed point, so the laser's origin stays put to millimetres while the dot moves centimetres. |
| Why not the magnetometer as truth? | It is disturbed by the motors and, at the presentation spot, by nearby steel (up to 14° peak-to-peak, slide 9 / backup). |

---

## Slide 9: Data: Magnetometer Yaw Aid (MAG TEST)

### The 30-second version
> "Sensor fusion with the magnetometer was part of our objective, so we built a heading aid that only trusts the field
> when the wheels are in a known state, and tested it with a self-grading test on the clamped table. The first run
> passed 36 of 37 steps over 57 minutes, where the gyro alone drifted almost 6 degrees an hour. But every new wheel
> pattern costs up to a degree of reference error, a second run failed, and the presentation spot bends the field, so
> the demo runs on the gyro with the aid switched off."

### Walking the plot
- **Top:** yaw error with the table clamped (true heading 0). Blue: on board, aid on, within the ±1° band; the spikes at
  1, 20 and 22 minutes are deliberate ±10° kicks to the estimate that the aid pulls back. Red: same data replayed with
  the gyro only, drifting to −5°.
- **Bottom:** the test step (stopped, 14 wheel direction patterns at three speeds, kicks, a 30-minute drift run).
- **MCEN:** run 1 (learning references in place) 36/37, worst 1.02°; run 2 (predicting new patterns from a model)
  stopped at 10/15 because the motor-magnet model moved about 1° between sessions; each new pattern costs 0.3–1.2°.

### Likely questions
| Question | Answer |
|---|---|
| Why do the wheels disturb the magnetometer so much? | Each motor has permanent magnets a few cm from the IMU; their field depends on where each shaft stops (up to ±10° offsets) and adds fast noise when spinning. We mapped it (MAG MAP) and only trust known wheel states. |
| Why not move the IMU away from the motors? | Not possible on this table (mounting fixed); the firmware works around it. |
| So is the magnetometer part of your fusion? | Built and validated on the clamped table, and selectable (`MAG ON`); for the demo it is off because the room field at that spot is distorted. The gyro after a proper calibration holds within 0.25° for 5 minutes, which meets requirement 1 without it. |
| What would fix it? | An absolute reference that the motors don't disturb: our future work uses AprilTags as a star sensor. |

---

## Slide 10: Data: Balancing the Table

### The 30-second version
> "The table is a pendulum: its centre of mass sits below the bearing's pivot, so gravity pulls it back to a rest tilt.
> Holding any other tilt costs the wheels a steady torque they can only absorb by speeding up. Each dot is one rest-tilt
> reading after a trim move; we brought the table to within a degree of level. Then we tried to raise the centre of mass
> to about a millimetre below the pivot for tilted slews, and the table became unpredictable, so the demo uses a stiff,
> repeatable setup and yaw-only slews."

### Item by item
- **Plot (10-31 session):** roll (blue) and pitch (gold) at rest after each move; dashed lines mark events (IMU glued,
  LEVEL ZERO, mass shifts, weights removed). Final ≈ −0.9 / −0.7°.
- **Physics:** gravity stiffness k = m g L (L = CG depth below the pivot). Swing period T = 2π √(I/k), so a longer swing
  means a CG closer to the pivot. At L ≈ 10 mm, k ≈ 0.5 N·m/rad ≈ 9 mN·m per degree held.
- **11-01:** raising the CG to ≈ 3 mm (top weights) made trims land 2–4× further than predicted and the table tip; the
  demo setup is stiff (CG ≈ 10 mm, swing ≈ 3.2–3.6 s), rest tilt −2.5 / +0.1°.
- **Next:** rigid, repeatable trim masses and CG ≲ 1 mm before tilted targets (future work: automatic balancer).

### Likely questions
| Question | Answer |
|---|---|
| How do you measure the CG height? | From the free-swing period: L = I ω² / (m g) (`tools/balance.py`), checked against the rest-tilt change per gram·millimetre of trim. |
| Why not just make it neutral? | At a few millimetres every loose cable or battery shift moves the rest tilt by degrees, and near zero the table can tip over the edge of the cup. It needs a rigid layout or active balancing. |
| Does the tilt affect heading control? | Barely: the slews turn about the vertical and roll/pitch are only damped at the rest tilt. Gravity has no torque about the vertical. |

---

## Slide 11: Data: Heading Holds by Laser (Req. 1)

### The 30-second version
> "This is requirement 1 measured against truth. The red line is a hold calibrated only five minutes after power-on: the
> gyro's yaw bias was still changing as it warmed up, and the table drifted almost a degree in two and a half minutes.
> Blue and green are after a ten-minute warm-up and a three-minute calibration: a quarter-degree band for five minutes,
> no trend."

### Walking the plot
- Each line is a DEMO LEVEL hold, plotted from its first laser reading; green band = ±1°.
- **Red:** −0.78° in 2.5 min ≈ 15 °/h: the BNO085's z bias shifts ~0.007 °/s over the first ~5 minutes after power-on.
- **Blue (5 min):** −0.20 to −0.45° band (0.25°), no trend; the small wander includes wheel-pattern switches.
- **Green (live run, 3 min):** within 0.15°.
- **Procedure now:** power on, 10 min warm-up, `CALIBRATE 180` with the table clamped.

### Likely questions
| Question | Answer |
|---|---|
| Why does the warm-up matter? | The MEMS gyro's bias changes with temperature as the chip warms; the calibration averages the bias, so it must be taken once it has settled. Gravity can't correct yaw bias, so whatever is left becomes drift. |
| How long can it hold? | Heading drift is no longer the limit; yaw drag fills the wheels in about 5 minutes. |
| What's the wander? | Mostly reading resolution (0.05°) and small kicks when the wheel-bias planner switches pattern. |

---

## Slide 12: Data: Estimator Heading Leak (Unexpected)

### The 30-second version
> "This is the most important thing we found this week. Every full-turn slew passed 4 of 4 by the firmware's own
> grading, while the laser said the table ended up to 2.3 degrees off, and by different amounts each run. Turning the
> other way flipped the sign, which pointed at rotation being lost. Integrating the logged gyro showed 363 to 364 degrees
> where the estimator said 360: the gyro was right and the estimator was removing about one percent of every turn.
> The cause was the accelerometer update (slide 6). After the fix: 0.05 degrees."

### Walking the plot
- **Left:** laser error after each full-turn SLEW YAW on 11-01, in time order: −0.10, +1.10, +0.06, +1.70, +2.28 (CCW),
  −1.80 (the mirrored CW test), then +0.05 after the fix (green).
- **Right:** rotation beyond 360° by three measures: grey = logged gyro integrated (3–4°), red = estimator (0),
  blue = laser (1.7–2.3°). After the fix all three agree.
- **Why it scattered:** the leak's size depends on the filter's tilt/heading cross-covariance, which depends on each
  run's tilt history.

### Likely questions
| Question | Answer |
|---|---|
| Why wasn't this caught earlier? | The estimator graded its own runs, and the magnetometer was disturbed. Only an independent reference (the laser) showed the disagreement. Biggest lesson of the project. |
| Is the remaining 0.05° real? | It is at the reading resolution; the replay of the same data shows a common ~0.3° bias contribution over other runs, so expect about ±0.3° run to run. |
| How do you know the fix is right, not just tuned? | Two independent checks: the gyro integral now matches the estimator (360.01° vs 360.0°), and the laser matches both. The projection is principled (gravity has no heading), and the scale fit used both turn directions. |

---

## Slide 13: Data: First-Run Start Failures (Unexpected)

### The 30-second version
> "We lost hours to what looked like wiring faults: at the first arm after every boot, all four wheels were flagged dead
> and one or two ran away. The pattern was in our log: it happened on the first run after a boot, seven times out of
> seven, and never after. The 100 Hz log shows why: for most of a second the wheel loop stopped while the encoders kept
> counting. The SD card's first write to a new log file searches for free space and blocked the main loop. When the loop
> resumed it saw one huge step and the detector flagged everything."

### Walking the plot (`DEMO_0016`, 21:59)
- **Top:** encoder counts jump ~1000 (2.5 wheel turns) between rows while...
- **Middle:** the measured speed stays at zero, and...
- **Bottom:** the motor duty stays frozen. Shaded: loop() blocked ≈ 0.8 s; the rows were logged in a burst afterwards
  (the IMU kept being read in the background, so the log has timestamps for every sample).
- **Fix:** the log file is opened and its first sectors written before arming (wheels still stopped); a wheel-loop step
  longer than 50 ms skips the fault checks and uses a normal step. The next first-arm run (DEMO ALL) started cleanly.
- **Still to do:** routine SD writes still pause the wheel loop ~0.1 s about once a second; moving the wheel loop to a
  timer interrupt removes that (firmware frozen until after this presentation).

### Likely questions
| Question | Answer |
|---|---|
| Why did wheels run away? | During the stall the motor kept its last duty, and after it the controller processed the queued samples with stale speeds; the dead-motor coast then holds the wheel at its speed. |
| Why didn't the IMU time out? | The IMU is read from `yield()`, which the SD library calls while waiting, so samples were queued, not lost. |
| Are the remaining pauses a risk? | They were present in every earlier run (116 in one 8-minute log) and the runs passed; the wheel loop now ignores them for fault decisions. The timer interrupt is the clean fix. |

---

## Slide 14: DEMO ALL: The Whole Run

### The 30-second version
> "This is the whole demonstration in one plot, from one command. Blue: a heading hold, graded for 60 seconds. Green:
> four slews through a full turn, each held five seconds. Orange: ten seconds later wheel B's motor is cut; its speed
> drops to zero and A, C and D take over. Nothing is reset between the parts, and the Teensy grades each one. This was
> the first arm after a boot, the case that always failed before."

### Walking the plot (`DEMO_0020`, 11-01 23:13)
- **Top:** heading from start: flat (hold), four smooth steps to 360°, flat (failure and hold). Reference dashed on top.
- **Middle:** roll and pitch stay within about a degree of the rest tilt throughout (damping only).
- **Bottom:** wheel speeds: ±10–20 rad/s hold bias, swings during the slews, B to zero after the cut (dotted line).
- **The sequencer (`full_demo.cpp`, backup code slide):** DEMO LEVEL → hand over to SLEW YAW without disarming, with
  targets relative to the heading the hold kept → FAULT TEST after the slews → one summary line:
  `DEMO ALL COMPLETE: 1 heading hold PASS (max 0.04 deg), 2 slews 4/4 PASS, 3 wheel failure wheel B PASS`.

### Likely questions
| Question | Answer |
|---|---|
| Why one command? | Stopping and restarting between tests braked the wheels and kicked the table off its zero; chaining them keeps the momentum and the reference continuous, and the demo is repeatable. |
| How long does it take? | About 5 minutes including settling; limited by yaw drag filling the wheels. |
| Requirement 1 slide? | Its numbers are on slide 3 and the laser holds (slide 11); the plot itself is a flat line (backup). |

---

## Slide 15: DEMO ALL: Requirement 2 (Attitude Truth)

### The 30-second version
> "After a full turn, the table came back to within five hundredths of a degree of where it started, by the laser.
> Before the estimator fix the same test ended 2.3 degrees off one way and 1.8 the other. Every one of the four targets
> was reached and held within a tenth or so of a degree."

### Walking the plot
- **Left:** laser error after 360°: two earlier runs before the fix (red) and this run (green, +0.05°, the bar shows the
  +0.02 to −0.03° reading range). Band = ±1°.
- **Right:** worst yaw error in each 5 s target hold: 0.03, 0.02, 0.13, 0.05°, against the 1° limit.
- **Honest comparison (say it if asked):** replaying this run's data through the old estimator gives 0.46° at the end of
  the slews; on earlier runs it was 1.7–2.3°. The leak varied run to run; the fix removes it.

### Likely questions
| Question | Answer |
|---|---|
| Why a full turn? | A heading error from a scale or estimator problem grows with the angle turned; a full 360° is the hardest case and ends where it started, so the laser can read it. |
| What about roll and pitch during the slews? | Within 0.23° of the rest tilt in the target holds (graded on board). |

---

## Slide 16: DEMO ALL: Requirement 3 (Wheel Failure)

### The 30-second version
> "The last part. Wheel B's motor is cut and the controller is not told. Half a second later the detector takes B out of
> the allocation; the attitude error peaks just under a degree and is back within one degree half a second after the
> failure; then 30 seconds on three wheels with at most 0.36°. On the wall the laser moved three tenths of a degree and
> stopped."

### Walking the plot
- **Top:** controller attitude error (roll, pitch, yaw) around the cut (dotted) and the detection (dashed, 0.50 s).
- **Bottom:** B drops to zero (coasting with the motor dead); D speeds up, then A, C and D share the load.
- **Grades:** detected ≤ 1 s ✔, recovered ≤ 10 s ✔ (0.5 s), held ±1° for 30 s ✔ (0.36°).
- **The wiggle before the cut:** the table is still settling from the last slew (the failure comes 10 s after the slews
  end); it stays well inside ±1°.

### Likely questions
| Question | Answer |
|---|---|
| Why is the laser peak smaller than the firmware's? | They measure different things: 0.90° is the controller's error against its reference during the transient; 0.3° is the table's heading change read by eye on the wall as the dot moved. Both are under 1°. |
| What if it failed mid-slew? | Simulated: detected and recovered in all 9 cases including mid-slew (backup). |

---

## Slide 17: Lessons Learned

### The 30-second version
> "Five lessons, each with the evidence that taught it. The first is the one we'd tell any future team: measure truth
> independently. The estimator said our slews were perfect while the table was 2.3 degrees off."

### Item by item
1. **Measure truth independently.** Estimator said 0.1°, table was 2.3° off.
2. **Warm up 10 min, calibrate 180 s.** Early calibration drifted ≈ 15 °/h.
3. **Keep the SD card out of the control path.** SD stalls froze the wheel loop; next: timer-interrupt wheel loop.
4. **Solder and strain-relieve every connector.** Wheel D's encoder lead and the motor-switch leads cost the most table
   time.
5. **Balance with a rigid, repeatable mass layout, then tilted slews.** Loose mass made trims unpredictable.
- **Demo checklist:** motor switch and pack leads checked; power on, 10 min warm-up, `CALIBRATE 180` clamped; DEMO ALL
  with laser readings after part 1, part 2 and the failure.
- **Improvements beyond KestrelSAT:** air bearing table; automatic wheel-failure detection tested for real; one-command
  self-grading demo; laser heading truth and log replay on the PC.

---

## Slide 18: Future Work: Next Phase

### The 30-second version
> "Three directions. Replace the PD controller with model-predictive control on the Teensy, so the wheel limits become
> hard constraints and a wheel failure is just a constraint change. Build an automatic balancer that moves trim masses
> until the gravity torque is zero, which removes the pendulum and unlocks tilted targets. And add a camera with
> AprilTags as a star sensor: absolute 3-axis attitude that bounds gyro drift and replaces the magnetometer."

### Item by item
- **TinyMPC control:** TinyMPC is an open-source model-predictive control solver designed for microcontrollers (ADMM
  based, fixed memory). On the Teensy 4.1 at 100 Hz it would solve a short-horizon optimal control problem every 10 ms.
  - Constraints built in: wheel speed and torque limits are respected by construction; on three wheels, re-solve with
    the reduced allocation.
  - Comparison plan: robustness to inertia error, disturbance torque and a live failure; rise and settling time vs the
    PD baseline (1.2–1.7 s rise, ≤ 4% overshoot); solve time per 10 ms step.
- **Automatic balancer:** motorized trim masses; estimate the CG offset from the gravity torque the wheels absorb, move
  the masses until it is zero. Removes the pendulum (today CG ≈ 10 mm below the pivot), unlocking tilted targets and
  full rotation about all three axes.
- **AprilTag star sensor:** fiducials at surveyed room positions act as known stars; a camera fix gives absolute
  attitude, fused in the MEKF alongside gyro and accelerometer. Independent truth for roll and pitch too (the laser gives
  heading only).

### Likely questions
| Question | Answer |
|---|---|
| Why would MPC beat the PD? | The PD knows nothing about limits; it can demand torque the wheels can't give. MPC plans with the limits and can trade error for momentum, which matters on a table that fills its wheels. |
| Can the Teensy run it? | TinyMPC targets much smaller microcontrollers; the Teensy's 600 MHz Cortex-M7 with an FPU has margin, but solve time per step is one of the things the plan measures. |
| How would the balancer estimate the CG? | The wheels' momentum change while holding a tilt is the gravity torque (no other x/y torque on the bearing); torque ÷ (m g) gives the CG offset vector. Our SLEW TILT firmware already maps the rest tilt over a full turn and drains wheel momentum with gravity, the same physics. |

---

## Slide 19: GenAI Use Documentation
State plainly what the tool did and what the team did.
- **Tool:** Claude Code (Opus model).
- **Firmware:** drafted the estimator and magnetometer aid, controller, allocation, fault detector, IMU driver, logger,
  command set, the self-grading tests and DEMO ALL, from the team's requirements and design decisions; the team
  reviewed, uploaded and tested every build.
- **Simulation and analysis:** truth models, log replay and balance tools, plots; traced the heading leak and the
  start-up stall from the logs. The team ran every test, read the laser and checked the conclusions.
- **Team-owned:** hardware, wiring and power, requirements and design decisions, test design and execution, final review.
- **Record:** `PROGRESS_LOG.md` and the git history.

Likely question: "How do you know the code is right?" Every build was tested on hardware against independent truth
(laser, hand-turn encoder counts, the level), simulated against a separately written model, and logged at 100 Hz so
results can be replayed and checked.

---

## Backup slides: what to pull up for which question
| If asked about... | Backup slide |
|---|---|
| The objective / overview | Overview/Objectives |
| The headline numbers in one place | Bottom Line: One Command, All Requirements |
| How the magnetometer update works | Code: Heading Update (Estimation) |
| Controller code | Code: Control Law & Wheel Allocation |
| How DEMO ALL chains the parts | Code: DEMO ALL Sequencer |
| Hardware architecture, power, photos | Build Progress: Hardware (2 slides) |
| What we did when | Progress: Software Build & Tests (2 tables) |
| Simulation | Simulation: Methods; Wheel Failure (9/9); 4-Attitude Slew |
| Wheel speed loop quality | Data: Wheel Speed Control |
| Yaw drift and calibration | Data: Attitude Determination at Rest |
| IMU sample loss | Data: IMU Data Integrity |
| Why the magnetometer is hard | Data: Magnetometer vs. Wheel Motors; Magnetometer at the Presentation Spot |
| The pendulum discovery | First DEMO SPIN; DEMO SPIN Leveling (+ photos, video); DEMO LEVEL Hold |
| The dead Teensy | Data: Flight Computer Failure |
| False wheel faults at rest | Data: False Dead-Motor Flags |
| Controller step response, inertia | Data: STEP TEST 1 (hard iron) and STEP TEST |
| Wheel failure on the table before DEMO ALL | Data: FAULT TEST on the Air Bearing; Wheel D Encoder Lead (real failure) |
| Why no tilted targets | Tilted Slews vs. Gravity; SLEW YAW on the Air Bearing; SLEW TILT |
| Requirement 1 plot from DEMO ALL | DEMO ALL: Requirement 1 |
| Overall status | Where We Stand |

---

## Harder questions to rehearse
| Question | Answer |
|---|---|
| What would you do differently from day one? | Set up the laser truth first, and keep the SD card off the control loop. Both would have saved days. |
| What is your biggest remaining risk for a live demo? | Connectors (wheel D's encoder lead, the motor-switch leads) and forgetting the warm-up. The checklist covers both; the firmware catches a real wheel failure if one happens. |
| Did you meet "all 3 axes" for Req. 2 during motion? | Heading by laser during motion; roll and pitch by the accelerometer (exact at rest, gated when accelerating). An independent 3-axis truth during motion is future work. |
| Why trust a 60 s hold for requirement 1? | The requirement asks for 1 minute; we also showed 5 minutes by laser within 0.25°. Beyond ~5 minutes the wheels fill from yaw drag, a physics limit without momentum dumping. |
| What is the yaw drag? | A small torque from the air flow in the bearing, about 0.1 mN·m; the wheels absorb it. Real satellites dump momentum with magnetorquers or thrusters. |
| What were the wheel speed loop's problems? | Wheel A rubs and C oscillates ±10–16 rpm with the same gains that hold B and D within ±1 rpm: hardware, not tuning (prelim data, backup). |

## If you only remember five things
1. **All three requirements were verified in one run, against a laser**, not just the estimator's own word.
2. **Gravity carries no heading.** Our filter was letting it change heading anyway during turns; projecting that out
   fixed a 1% loss per turn.
3. **Four wheels in a pyramid: any three span all axes.** The detector compares the push with the response, without a
   current sensor, and drops a dead wheel in about half a second.
4. **Wheels move momentum, they don't remove it.** Gravity on a pendulous table and yaw drag fill them; balance and
   hold time follow from that.
5. **The bugs that mattered were invisible from inside.** Independent truth (laser) and complete logs (100 Hz, replayable)
   are what found them.
