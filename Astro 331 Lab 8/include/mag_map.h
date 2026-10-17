#pragma once
#include <stdint.h>

// "MAG MAP": scripted wheel-speed schedule for mapping how the reaction wheels disturb the
// magnetometer. Run with the table FIXED (not floating): single wheels spinning push on the table.
//
// Schedule (each step held kMagMapStepS; the analysis averages the settled part of each step):
//   all wheels:   0, +50, +100, +200, 0, -50, -100, -200, 0 rpm
//   each wheel A..D alone: +100, +200, 0, -100, -200, 0 rpm (others at 0)
// => 33 steps, ~11 min at 20 s/step. The log's test_step column carries the step index, so the
// field can be averaged per wheel state (speed, direction, which wheel) afterwards.
namespace mag_map {

struct Step {
  int16_t rpm[4];  // A, B, C, D
};

constexpr Step kSteps[] = {
    // all wheels together
    {{0, 0, 0, 0}}, {{50, 50, 50, 50}}, {{100, 100, 100, 100}}, {{200, 200, 200, 200}}, {{0, 0, 0, 0}},
    {{-50, -50, -50, -50}}, {{-100, -100, -100, -100}}, {{-200, -200, -200, -200}}, {{0, 0, 0, 0}},
    // A alone
    {{100, 0, 0, 0}}, {{200, 0, 0, 0}}, {{0, 0, 0, 0}}, {{-100, 0, 0, 0}}, {{-200, 0, 0, 0}}, {{0, 0, 0, 0}},
    // B alone
    {{0, 100, 0, 0}}, {{0, 200, 0, 0}}, {{0, 0, 0, 0}}, {{0, -100, 0, 0}}, {{0, -200, 0, 0}}, {{0, 0, 0, 0}},
    // C alone
    {{0, 0, 100, 0}}, {{0, 0, 200, 0}}, {{0, 0, 0, 0}}, {{0, 0, -100, 0}}, {{0, 0, -200, 0}}, {{0, 0, 0, 0}},
    // D alone
    {{0, 0, 0, 100}}, {{0, 0, 0, 200}}, {{0, 0, 0, 0}}, {{0, 0, 0, -100}}, {{0, 0, 0, -200}}, {{0, 0, 0, 0}},
};
constexpr int kNumSteps = sizeof(kSteps) / sizeof(kSteps[0]);

}  // namespace mag_map
