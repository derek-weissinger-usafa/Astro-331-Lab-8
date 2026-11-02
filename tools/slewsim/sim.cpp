// Host simulation of SLEW TEST / SLEW YAW / SLEW TILT on a pendulous air-bearing table, using the
// firmware's slew_test.cpp, attitude_ctrl.cpp, tune.cpp and wheel_geometry.h. Ideal wheel torques
// (no speed loop), perfect attitude/rate estimate.
// Usage: sim [yaw|fixed|tilt] [roomTorque_mNm] [trace.csv|-] [L_mm] [tiltScale] [kFactor] [gyroBias_degs]
//   L_mm       centre of mass below the bearing centre (default 13; the table on 10-31)
//   tiltScale  multiplies the SLEW TILT target offsets (default 1: +-2 deg class targets)
//   kFactor    GRAV_K given to the firmware / true stiffness (default 1: exact)
//   gyroBias   x/y rate bias in the controller's rate (deg/s; y gets 0.6x), as DEMO_0012 (~0.05)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "attitude_ctrl.h"
#include "control_params.h"
#include "slew_test.h"
#include "tune.h"
#include "wheel_geometry.h"

using namespace wheel_geometry;

static Vec3 matVec(const Mat<3, 3>& A, const Vec3& v) {
  return {A(0, 0) * v.x + A(0, 1) * v.y + A(0, 2) * v.z, A(1, 0) * v.x + A(1, 1) * v.y + A(1, 2) * v.z,
          A(2, 0) * v.x + A(2, 1) * v.y + A(2, 2) * v.z};
}

int main(int argc, char** argv) {
  const char* modeArg = argc > 1 ? argv[1] : "fixed";
  const SlewTest::Mode mode = !strcmp(modeArg, "yaw") ? SlewTest::Mode::YawOnly
                              : !strcmp(modeArg, "tilt") ? SlewTest::Mode::Tilt
                                                        : SlewTest::Mode::Absolute;
  const float room = (argc > 2 ? atof(argv[2]) : 2.0f) * 1e-3f;  // N m, fixed in the room
  FILE* trace = argc > 3 && strcmp(argv[3], "-") ? fopen(argv[3], "w") : nullptr;
  const float L = (argc > 4 ? atof(argv[4]) : 13.0f) * 1e-3f;
  const float tiltScale = argc > 5 ? atof(argv[5]) : 1.0f;
  const float kFactor = argc > 6 ? atof(argv[6]) : 1.0f;
  const float gbias = (argc > 7 ? atof(argv[7]) : 0.0f) * kDegToRad;
  const Vec3 gyroErr = {gbias, 0.6f * gbias, 0.0f};
  if (trace) fprintf(trace, "t_s,yaw_deg,yaw_ref_deg,roll_deg,pitch_deg,wA,wB,wC,wD,phase,target\n");
  const bool tiltTable = L < 0.0125f;  // the balanced-up table carries ~220 g more on top
  const float I[3] = {tiltTable ? 0.155f : 0.14f, tiltTable ? 0.155f : 0.14f, 0.0776f};
  tune::inertia[0] = I[0];  // SET IXX/IYY/IZZ as measured
  tune::inertia[1] = I[1];
  tune::inertia[2] = 0.0776f;
  const float m = tiltTable ? 5.2f : 5.1f, g = 9.81f;
  // Small horizontal offset: ~1 deg of rest tilt at L = 13 mm, scaled with L so the rest tilt stays
  // ~1 deg (the table is trimmed level at whatever height it runs).
  const Vec3 rcg = {0.00004f * L / 0.013f, 0.00027f * L / 0.013f, -L};
  const float kTrue = m * g * L;
  tune::gravK = kTrue * kFactor;
  const float J = params::kWheelInertia;

  Quat q = qFromEuler321(-1.2f * kDegToRad, -0.2f * kDegToRad, 0.3f);
  Vec3 w = {0, 0, 0};
  float ws[kN] = {0, 0, 0, 0};
  bool en[kN] = {true, true, true, true};

  SlewTest slew;
  if (mode == SlewTest::Mode::Absolute)
    for (int i = 0; i < 4; i++) slew.setTarget(i, -1.2f, -0.2f, params::kSlewTargets[i][2]);
  if (mode == SlewTest::Mode::Tilt)
    for (int i = 0; i < 4; i++)
      slew.setTiltTarget(i, params::kSlewTiltTargets[i][0] * tiltScale, params::kSlewTiltTargets[i][1] * tiltScale,
                         params::kSlewTiltTargets[i][2]);
  AttitudeController ctrl;
  const float dt = 0.01f;
  // Let the free table settle first (no control), so the test starts from rest.
  const float settleDamp = 0.02f * sqrtf(I[0] * kTrue / (0.14f * 0.64f));
  for (int k = 0; k < 6000; k++) {
    const Mat<3, 3> A = qToMatrix(q);
    const Vec3 gb = matVec(A, {0, 0, -g * m});
    Vec3 tau = cross(rcg, gb) + matVec(A, {room, 0.5f * room, 0}) - w * settleDamp;
    const Vec3 Iw = {I[0] * w.x, I[1] * w.y, I[2] * w.z};
    tau = tau - cross(w, Iw);
    w = w + Vec3{tau.x / I[0], tau.y / I[1], tau.z / I[2]} * dt;
    q = qnormalize(qmul(qconj(qFromRotVec(w * dt)), q));
  }
  w = {0, 0, 0};
  const Vec3 e0 = qToEuler321(q);
  printf("start: roll %.2f pitch %.2f deg, L %.1f mm (k %.3f N m/rad, swing %.1f s), room torque %.1f mN m, mode %s\n",
         e0.x * kRadToDeg, e0.y * kRadToDeg, L * 1e3f, kTrue, 2 * 3.14159f * sqrtf(I[0] / kTrue), room * 1e3f, modeArg);
  if (mode == SlewTest::Mode::Tilt) {
    const float cap = SlewTest::tiltCapacity();
    for (int i = 0; i < 4; i++) {
      const SlewTest::Target& tg = slew.tiltTarget(i);
      const float h = slew.tiltBudget(i, tune::gravK);
      printf("  target %d: %+.1f/%+.1f deg at %+.0f: budget %.1f of %.1f mN m s (%s)\n", i + 1, tg.rollDeg, tg.pitchDeg,
             tg.yawDeg, h * 1e3f, params::kTiltBudgetFrac * cap * 1e3f,
             h > params::kTiltBudgetFrac * cap ? "firmware would REFUSE" : "ok");
    }
  }

  slew.start(q, mode, tune::gravK);
  ctrl.setTrajectory(slew.refQ(), slew.refW(), slew.refA());
  float wmax = 0, tiltDevMax = 0;
  double t = 0;
  int doneAt = -1;
  for (int k = 0; k < 60000; k++, t += dt) {
    Vec3 h = {0, 0, 0};
    for (int i = 0; i < kN; i++) h = h + axis(i) * (J * ws[i]);
    const SlewTest::Event ev = slew.step(dt, q, h);
    ctrl.setTrajectory(slew.refQ(), slew.refW(), slew.refA());
    ctrl.setFeedforward(slew.gravityFF());
    const Vec3 tauC = ctrl.update(q, w + gyroErr, dt, h);
    float u[kN];
    allocate(tauC, en, u);
    Vec3 tauW = {0, 0, 0};
    for (int i = 0; i < kN; i++) {
      ws[i] += u[i] / J * dt;
      tauW = tauW - axis(i) * u[i];
      if (fabsf(ws[i]) > wmax) wmax = fabsf(ws[i]);
    }
    const Mat<3, 3> A = qToMatrix(q);
    const Vec3 gb = matVec(A, {0, 0, -g * m});
    Vec3 tau = cross(rcg, gb) + matVec(A, {room, 0.5f * room, 0}) + tauW;
    tau.z += -5e-5f * (w.z > 0 ? 1.0f : -1.0f) - 2e-4f * w.z;  // yaw drag
    const Vec3 Iw = {I[0] * w.x, I[1] * w.y, I[2] * w.z};
    tau = tau - cross(w, Iw + h);
    w = w + Vec3{tau.x / I[0], tau.y / I[1], tau.z / I[2]} * dt;
    q = qnormalize(qmul(qconj(qFromRotVec(w * dt)), q));

    if (trace && k % 10 == 0) {
      const Vec3 e = qToEuler321(q);
      fprintf(trace, "%.2f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%c,%d\n", t, e.z * kRadToDeg, slew.refYawDeg(),
              e.x * kRadToDeg, e.y * kRadToDeg, ws[0], ws[1], ws[2], ws[3], slew.phaseChar(), slew.index());
    }
    const Vec3 er = slew.errorDeg();
    if (slew.phase() == SlewTest::Phase::Hold || slew.phase() == SlewTest::Phase::Done) {
      const float d = fmaxf(fabsf(er.x), fabsf(er.y));
      if (d > tiltDevMax) tiltDevMax = d;
    }
    if (ev == SlewTest::Event::ScanDone) {
      const float* r = slew.restRollFitDeg();
      const float* p = slew.restPitchFitDeg();
      printf("%7.1f s  scan: rest roll %.2f %+.2f cos %+.2f sin, pitch %.2f %+.2f cos %+.2f sin, wheels %.1f %.1f %.1f %.1f\n",
             t, r[0], r[1], r[2], p[0], p[1], p[2], ws[0], ws[1], ws[2], ws[3]);
    }
    if (ev == SlewTest::Event::UnloadDone)
      printf("%7.1f s  target %d unloaded: |h_xy| %.1f -> %.1f mN m s in %.1f s\n", t, slew.index() + 1,
             slew.hxyAtUnloadStart() * 1e3f, sqrtf(h.x * h.x + h.y * h.y) * 1e3f, slew.lastUnloadS());
    if (ev == SlewTest::Event::Settled) printf("%7.1f s  target %d reached\n", t, slew.index() + 1);
    if (ev == SlewTest::Event::SettleTimeout) printf("%7.1f s  target %d NOT reached\n", t, slew.index() + 1);
    if (ev == SlewTest::Event::TargetDone || ev == SlewTest::Event::Complete) {
      const int i = ev == SlewTest::Event::Complete ? 3 : slew.index() - 1;
      const SlewTest::Result& r = slew.result(i);
      const Vec3 e = qToEuler321(q);
      printf("%7.1f s  target %d %s: max err roll %.3f pitch %.3f yaw %.3f deg (rest %.2f, %.2f)  tilt now %.2f %.2f  wheels %.1f %.1f %.1f %.1f\n",
             t, i + 1, r.pass ? "PASS" : "FAIL", r.maxErrDeg.x, r.maxErrDeg.y, r.maxErrDeg.z, r.restRollDeg,
             r.restPitchDeg, e.x * kRadToDeg, e.y * kRadToDeg, ws[0], ws[1], ws[2], ws[3]);
    }
    if (ev == SlewTest::Event::Complete) doneAt = k;
    if (wmax > params::kDemoWheelAbortRadS) {
      printf("%7.1f s  WHEEL LIMIT (%.0f rad/s) in target %d (phase %c)\n", t, wmax, slew.index() + 1, slew.phaseChar());
      break;
    }
    if (doneAt >= 0 && k - doneAt > 6000) break;  // hold target 4 for 60 s
  }
  printf("result: %d/4 PASS, peak wheel %.1f rad/s, worst roll/pitch err in holds %.3f deg, t %.0f s\n",
         slew.passCount(), wmax, tiltDevMax, t);
  return 0;
}
