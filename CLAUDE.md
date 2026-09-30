# CLAUDE.md

Guidance for Claude Code when working in this repository.

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

Target hardware: **Teensy 4.1** (ARM Cortex-M7, 600 MHz, hardware FPU — `float` math is cheap; prefer `float` over `double`).

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

- Add library dependencies via `lib_deps` in `platformio.ini` rather than vendoring, unless the library needs local modification (then put it in `lib/`).
- Keep the control loop non-blocking: no `delay()` in `loop()`; use `micros()`/`elapsedMicros` timing so the sensor-fusion and control rates are deterministic.
- Keep sensor drivers, attitude estimation, control law, and wheel fault detection in separate modules so each can be tested independently.
- Angles in code are radians unless a name says otherwise (e.g. `yawDeg`); document frame conventions (body vs. reference) where quaternions/rotations are defined.
