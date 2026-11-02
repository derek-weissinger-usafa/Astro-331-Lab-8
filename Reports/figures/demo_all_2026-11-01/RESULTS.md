# DEMO ALL, 2026-11-01 23:13 (`test_runs/DEMO_0020.CSV`) — results for the final presentation

One command (`DEMO ALL`) ran all three requirements back to back in one armed run, with no disarm or reset
between parts: 1 heading hold, 2 SLEW YAW (4 targets, one full 360° turn), 3 wheel B motor failure.
Truth: a laser on the deck, read on a printed scale on a wall 2.48 m from the bearing centre
(0.1° ≈ 4.3 mm). Figures: `make_figs.py` in this folder (`python Reports/figures/demo_all_2026-11-01/make_figs.py`).

| Requirement | Firmware (estimate) | Laser (truth) | Limit |
|---|---|---|---|
| 1. Hold heading, no jitter/drift (60 s) | max error 0.04°, jitter 0.011° rms, drift 0.000° | steady at −0.05° | ±1° |
| 2. Attitude to ±1° (after a full 360° of slews) | 4 targets reached and held, max yaw error 0.03 / 0.02 / 0.13 / 0.05°, roll ≤ 0.23°, pitch ≤ 0.16° | −0.05° before → +0.02…−0.03° after: **≈ 0.05° error** | ±1° |
| 3. Detect a wheel failure and keep 1 and 2 | wheel B detected 0.50 s after its motor was cut; back within ±1° 0.5 s after the failure (peak 0.90°); 30 s hold max 0.36° on wheels A, C, D | peak −0.31° at the cut, then steady −0.18° | detect ≤ 1 s, recover ≤ 10 s, then ±1° |
| Stretch: 4 attitudes along the shortest path | SLEW YAW 4/4 PASS (yaw targets at the rest tilt) | as row 2 | ±1° |

Firmware summary line: `# DEMO ALL COMPLETE: 1 heading hold PASS (max 0.04 deg), 2 slews 4/4 PASS, 3 wheel failure wheel B PASS`

## Checks behind the numbers
- **Sensor data integrity:** 35,379 gyro samples; timestamps match the BNO085's own sequence counter (ratio 0.99974, no
  doubled or zero steps). Integrating the logged yaw rate over the run gives +360.01° for the full turn; the estimator says
  360.0°; the laser says 360.05°.
- **Replay:** re-running the log through the firmware's estimator on the PC (`tools/replay`) reproduces the flown heading
  within 0.015°.
- **The yaw fix, on this same data:** the old estimator (accelerometer update leaking into heading, scale 1.009) replayed
  on this run ends the slews 0.46° off. On earlier runs the same leak cost 1.7–2.3° (laser), because its size depends on
  each run's tilt history: that variability is what made it hard to see (PROGRESS_LOG §2.32).
- **Start-up fix:** this run was the first arm after a boot, the case that failed every time before tonight (SD card file
  creation stalled the wheel loop; PROGRESS_LOG §2.33). The file was now prepared before arming (~1.2 s, wheels stopped).

## Known limitation (shown honestly)
- The wheel loop still pauses for ~0.1 s about once a second (101 pauses, median 118 ms) while the SD card writes. They
  were already present in earlier runs (§2.34) and the run passed through them; the fix (wheel loop on a timer interrupt)
  is deliberately left until after the presentation, so the firmware that produced these results is unchanged.

## Figures
- `fig1_overview.png` — the whole run: heading (0 → 360°) with the three parts shaded, tilt, wheel speeds.
- `fig2_req1_hold.png` — requirement 1: heading error over the graded 60 s hold against the ±1° band.
- `fig3_req2_slews.png` — requirement 2: laser error after a full turn before vs after the yaw fix, and each target's error.
- `fig4_req3_fault.png` — requirement 3: attitude error and wheel speeds around the wheel B failure.
