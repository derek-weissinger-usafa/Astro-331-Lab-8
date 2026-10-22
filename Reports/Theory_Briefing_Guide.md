# Briefing guide: the two Theory slides

For presenting slides 5 and 6 of the preliminary data review
(`Astro 331 Lab 8_2_3 Body Problem_Prelim_Pres_FILLED.pptx`). About 2 minutes per slide.
Each section has a short script, an explanation of every item on the slide, and likely instructor
questions with answers. Code references are in the PlatformIO project (`Astro 331 Lab 8/`).

---

## Slide 5: Theory: Estimation & Control Loop

### The 30-second version (say this first)
> "This is our whole loop on one page. Every 10 milliseconds the IMU gives us a gyro sample. The
> estimator, a multiplicative extended Kalman filter, turns the gyro and accelerometer into our best
> guess of attitude. The controller compares that to where we want to be and computes a torque. The
> allocation step splits that torque across our four wheels. Each wheel has its own speed loop that
> drives the motor. The wheels push on the table, the table moves, and the IMU measures it, which
> closes the loop. Everything runs on the Teensy at 100 Hz."

### Walking the diagram, box by box

**1. BNO085 (yellow): the sensor**
- 9-axis IMU: gyroscope (rotation rate), accelerometer (acceleration; at rest that's just gravity),
  magnetometer.
- We read **raw** gyro and accelerometer at 100 Hz and do our own fusion. The chip can fuse internally,
  but that's a black box we can't tune or verify.
- "INT-driven": the chip raises a pin when new data is ready and the Teensy reads it right away. That
  keeps sample timing accurate (see `imu_bno085.cpp`).

**2. MEKF (first blue box): the estimator**
- Output: **q̂** (attitude as a quaternion) and **ω̂** (body rotation rate, gyro bias removed).
- "Gyro predict, accel update": the gyro moves the estimate forward every sample; the accelerometer
  corrects it. Details below.

**3. Attitude PD (second blue box): the controller**
- Input: the estimate plus a **reference** (blue arrow): where we *want* the table (`q_ref`), how fast
  it should turn (`ω_ref`), and how fast that rate should change (`α_ref`).
- `ARM`: reference is "level, heading zero, not moving". Demos: a smooth trajectory.
- Output: **τ**, the torque we want on the table (slide 6).

**4. Allocation (third blue box)**
- Turns one 3-axis torque into four wheel commands (four wheels, three axes). Slide 6.
- Output: **ω_w**, a speed setpoint per wheel.

**5. Wheel PI (pink box): the inner loop**
- One small controller per wheel: reads the encoder, adjusts PWM so the wheel tracks its speed setpoint.
- Also limits motor current (~2.5 A) without a current sensor, by keeping the drive voltage close to
  the motor's back-EMF.

**6. Table + wheels (grey box): the physics**
- `I ω̇ = −ḣ_w − ω × (Iω + h_w) + τ_dist`. In words: **the table's rotation changes opposite to the
  wheels' momentum.** Speed a wheel up one way and the table turns the other way.
  - `I`: table inertia. `h_w`: total wheel momentum.
  - `ω × (…)`: gyroscopic coupling between axes when things spin.
  - `τ_dist`: disturbances (gravity from imbalance, air drag).

**7. Supervisor (red text):** cuts the wheels on `KILL`, IMU data gap > 100 ms, tilt > 30°,
table rate > 2 rad/s, or wheel overspeed.

### The bullets: explaining the MEKF

**"Multiplicative EKF: 6 error states (3 attitude + 3 gyro bias), attitude kept as a quaternion"**
- **Kalman refresher:** blend a *model* (prediction) with *measurements* (correction), weighting each by
  trust. Uncertainty is tracked in the covariance matrix **P**.
- **Why "multiplicative"?** A quaternion is 4 numbers for 3 angles, and its length must stay 1. A normal
  Kalman filter adds corrections to the 4 numbers and breaks that rule. Instead we keep the quaternion
  separately and let the filter estimate only a **small 3-angle error** on top of it. After each
  correction we rotate the quaternion by that error and reset the error to zero.
- **Why 6 states?** 3 for the small attitude error, plus 3 for **gyro bias**: the small constant offset
  every gyro has, which would otherwise build up into drift.

**"Predict at 100 Hz with the gyro; update with the accelerometer (gravity direction) → corrects roll and pitch"**
- **Predict:** gyro rate minus bias estimate, rotate the attitude by rate × 10 ms. Uncertainty grows.
- **Update:** at rest the accelerometer measures gravity, i.e. which way is "up" in the table's frame.
  The filter compares that with where its estimate says "up" should be, and corrects.
- **"Rejects samples when |a| is not ≈ 1 g":** if the measured acceleration isn't within 0.15 g of
  9.81 m/s², the table is accelerating, not just tilted, so the update is skipped or down-weighted.

**"Yaw is gyro-only…"**
- **Gravity points straight down, so it tells you nothing about which way you're facing.** Turning
  about the vertical doesn't change the gravity vector. The accelerometer corrects roll and pitch,
  **never yaw**.
- So yaw is the integrated gyro, and any leftover z-gyro bias error becomes drift (bias error × time).
- The magnetometer (compass) would fix yaw, but the wheel motors disturb it (data slide 12). For now the
  z bias comes from averaging the gyro for 60–180 s while the table sits still at startup.

**"Frames: body z up, x = roll, y = pitch (right-handed); q maps inertial → body"**
- Body frame: fixed to the table, z up, x and y flat on the deck. Inertial frame: fixed to the room.
- Roll = rotation about x, pitch about y, yaw about z. Rolling about x tilts the table *along* y.

### Likely questions on slide 5
| Question | Answer |
|---|---|
| Why not use the BNO085's built-in fusion? | It's a black box: no tuning, no uncertainty, no control over how it treats the magnetometer, which the motors disturb. Our own filter lets us verify and tune each piece. |
| Why a quaternion instead of Euler angles? | Euler angles break down near ±90° pitch (gimbal lock). A quaternion is smooth everywhere; we convert to roll/pitch/yaw only for display. |
| How do you get the gyro bias? | Startup: average the gyro while still (60–180 s). After that the filter refines the x and y biases from the accelerometer. The z bias stays locked at the startup value: gravity can't observe it, and letting the filter adjust it made drift worse in our tests. |
| What's your yaw accuracy? | About 3 °/h drift after a 10-minute warm-up and 180 s calibration (replay of a static test): within ±1° for ~20 min. Roll and pitch don't drift: 0.02° noise at rest. |
| What happens when the table spins? | The gyro handles rotation. Centripetal acceleration could fool the accelerometer, but the 1 g gate rejects or down-weights those samples. |
| Why 100 Hz? | Well above the control bandwidth (2 rad/s ≈ 0.3 Hz), and within the IMU's and I²C bus's limits with three sensors. |

---

## Slide 6: Theory: Control Law & Wheel Allocation

### The 30-second version
> "The controller is a PD controller written for quaternions, with feed-forward terms so it can follow
> a moving target, not just hold still. Its output is a torque, which our four wheels have to produce.
> They sit in a pyramid at 45°, so any three of them can still control all three axes. That's the basis
> for our wheel-failure requirement. Each wheel then runs its own speed loop. The big physical limit is
> that reaction wheels can't get rid of momentum: a steady disturbance, like an imbalanced table, keeps
> speeding the wheels up until they hit their limit."

### The control law, term by term
`τ = −Kp θₑ − Kd (ω − ω_ref) + I α_ref + ω × (Iω + h_w)`   (`src/attitude_ctrl.cpp`)

- **`−Kp θₑ`: proportional (spring).** θₑ is the attitude error (how far, about which axis, in radians),
  computed from the **error quaternion**, so it works at any orientation and always takes the short way
  around. The further off, the harder we push back.
- **`−Kd (ω − ω_ref)`: derivative (damper).** Pushes against the difference between how fast we're
  turning and how fast we should be. Stops overshoot and oscillation. Uses the measured rate directly
  rather than differentiating the angle, which would be noisy.
- **`+ I α_ref`: feed-forward.** When the reference itself accelerates (e.g. spinning up in DEMO SPIN),
  the needed torque is known in advance (inertia × acceleration), so we apply it directly. The PD terms
  only correct mistakes.
- **`+ ω × (Iω + h_w)`: gyroscopic compensation.** When the table rotates while the wheels carry
  momentum, the momentum vectors swing around and create torque on other axes. This term cancels that,
  so each axis behaves as a simple independent system. Matters most when spinning with fast wheels.

**"Kp = I ωₙ², Kd = 2ζ I ωₙ (ωₙ = 2 rad/s, ζ = 0.9)"**
- With the coupling cancelled, each axis is `I θ̈ = τ`: a mass with a spring and damper. These formulas
  place it at a chosen **natural frequency ωₙ** and **damping ratio ζ** (textbook second-order system).
- **ωₙ = 2 rad/s:** settles in ~2 s. Gentle on purpose: the wheels' torque and momentum are small.
- **ζ = 0.9:** almost critically damped, essentially no overshoot.
- With the model inertia (0.0623 kg·m²): Kp ≈ 0.25 N·m/rad ≈ 4.3 mN·m per degree; Kd ≈ 0.22 N·m·s/rad.
- No integral term (Ki = 0): the wheels already integrate a steady torque by speeding up, and an
  integrator would only fill them faster.

### The wheels: "4 wheels in a 45° pyramid"   (`include/wheel_geometry.h`)
- Each spin axis tilts 45° up from horizontal and points outward: A toward +x, B +y, C −x, D −y.
- **Each wheel affects two axes:** A and C share **roll**, B and D share **pitch**, all four share **yaw**.
- **Why 4 for 3 axes?** Redundancy. Any 3 non-parallel axes span 3D, so **any 3 of the 4 wheels can
  still produce torque about every axis.** Lose a wheel and the allocation recalculates with three
  (requirement 3).

**"Minimum-norm allocation u = −D⁺τ over the enabled wheels"**
- D is a 3×4 matrix whose columns are the wheel axes: how wheel torques add up to a body torque.
- Four wheels, three axes means infinitely many combinations give the same torque. **D⁺ (pseudo-inverse)
  picks the one with the smallest total wheel effort** (least squares).
- The minus sign is reaction: to push the table one way, the wheel is torqued the opposite way.
- "Over the enabled wheels": a disabled wheel's column is dropped, and the same formula works with three.

**"The 4th-wheel null mode [+1, −1, +1, −1] holds a bias speed"**
- One combination produces **zero net torque**: A and C up, B and D down by the same amount; they cancel.
  This is the extra freedom a fourth wheel gives.
- DEMO SPIN uses it to keep the wheels at ~100 rpm without moving the table, away from zero speed where
  static friction and the magnetometer disturbance are worst. DEMO LEVEL turns it off for more headroom.

**"Inner loop: 100 Hz wheel-speed PI with a back-EMF current limit"**   (`include/wheel_speed_ctrl.h`)
- The controller commands a *torque*; the firmware converts it to a wheel *speed* setpoint (torque ÷
  wheel inertia, integrated), and each wheel's PI loop tracks that speed from its encoder.
- **Why a speed loop?** Each wheel has its own battery pack and they discharge unevenly, so the same PWM
  gives different torque per wheel; friction differs too (wheel A rubs). Closing the loop on measured
  speed cancels those differences.
- **Current limit without a sensor:** motor current ≈ (applied voltage − back-EMF), and back-EMF ∝ speed.
  Keeping duty close to "speed ÷ free speed" caps current at ~2.5 A.

**"Physics limit: wheels cannot dump momentum…"**
- **The most important idea on this slide.** Reaction wheels don't create momentum; they *move* it
  between table and wheels.
- Any **steady outside torque** (gravity on an imbalanced table) forces the wheels to absorb momentum at
  that rate until one hits its speed limit. Time ≈ capacity ÷ disturbance torque.
  Our capacity: ~24 mN·m·s per tilt axis. At the ~8 mN·m roll imbalance we measured: ~3 s, exactly what
  DEMO_0002 showed (data slide 14).
- Real satellites dump momentum with magnetorquers or thrusters. We can't, so **balance is everything**
  for roll and pitch. Yaw has no gravity torque, so it doesn't have this problem.

**The box under the picture**
- **Wheel J = 3.06×10⁻⁴ kg·m²:** from the CAD model (aluminium, 161 g).
- **Table I ≈ 0.062 kg·m²:** a *model* (5 kg hollow cube). Air-bearing data fits 0.05–0.13 kg·m².
- **60 rad/s limit:** ~573 rpm, our safety cap (the motors can go faster at 12 V).

### Likely questions on slide 6
| Question | Answer |
|---|---|
| How did you choose the gains? | Pole placement: chose ωₙ = 2 rad/s and ζ = 0.9; the formulas give Kp and Kd from the inertia. Tested in simulation against an independent rigid-body model before hardware. |
| Why no integral term? | Under a constant disturbance an integrator just saturates the wheels faster; the wheels already integrate torque. The real fix for a steady disturbance is balance. |
| Why a pyramid, not three orthogonal wheels? | Fault tolerance: three orthogonal wheels lose an axis when one fails; four in a pyramid keep all three axes with any three. |
| What does the pseudo-inverse do physically? | Picks, out of every way the four wheels could make the torque, the one with the least total wheel torque. |
| How will you detect a wheel failure? (planned) | Each speed loop knows the speed it asked for. A wheel that stops tracking (large, persistent error) is flagged, removed from the allocation, and control continues on three. |
| What limits how long you can hold attitude? | Momentum capacity vs disturbance torque. Roll imbalance fills wheel A in ~3 s today; trimming the balance is next. Heading hold is limited by drift, not saturation. |
| What's the gyroscopic term for? | When the table turns while the wheels hold momentum, the momentum vector rotates and makes cross-axis torque. The term cancels it so each axis is independent. |
| Why no D term in the wheel speed loop? | See below. |

### Why the wheel speed loop is PI, not PID
- **The plant doesn't need it.** From motor voltage to wheel *speed*, a DC motor behaves like a
  first-order system (one lag: the rotor and wheel spinning up against back-EMF). A PI controller on a
  first-order plant gives a well-damped response without a D term. D earns its place on *second-order*
  plants, like torque → attitude (a double integrator), which is why the attitude loop is PD.
- **D would amplify encoder noise.** The encoder gives 400 counts per wheel turn. Sampled every 10 ms,
  one count is a speed step of about 1.6 rad/s. A D term differentiates speed (i.e. uses acceleration),
  turning that ±1-count jitter into noise of order 150 rad/s². Filtering it enough to be usable adds lag,
  which cancels the benefit of D.
- **The overshoot we saw came from other causes, and those were fixed:**
  1. *Wrong feedforward* (Owen, 2026-10-08): the Manual-mode feedforward used a free speed half the
     measured value, doubling the feedforward and overshooting every speed change. Fixed with the
     measured value; later re-measured as ~160 rad/s per unit duty after the 6.3:1 gear correction.
  2. *Integrator wind-up* (Owen, 2026-10-13): during a setpoint ramp the integrator kept growing and
     overshot at the end. Now the integrator is held while ramping; bench overshoot roughly halved
     (+1.7 → +0.9 rad/s, −2.3 → −1.3 rad/s).
  3. *Step setpoints*: `SPEED` commands now ramp at 20 rad/s² instead of jumping.
  4. *Armed mode* has its own anti-wind-up: the speed setpoint can't lead the measured speed by more than
     13 rad/s, the integrator is clamped, and the duty is slew-limited.
- **What's left is hardware, not tuning:** wheels A and C oscillate ±10–16 rpm while B and D hold ±1 rpm
  with identical gains (A rubs and its pack is low; C unexplained). A D term wouldn't fix a mechanical
  rub; fixing the hardware will.

---

## If you only remember four things
1. **Gravity fixes roll and pitch; nothing fixes yaw** except the startup calibration (later the
   magnetometer). Hence ~3 °/h of yaw drift.
2. **The MEKF estimates a small 3-angle error on top of a quaternion.** That's what "multiplicative"
   means, and it keeps the quaternion valid.
3. **The controller is a spring + damper on each axis**, plus terms that cancel known effects
   (feed-forward and gyroscopic).
4. **Wheels move momentum, they don't remove it.** Any steady torque, like imbalance, eventually saturates
   them, so balance limits our hold time.
