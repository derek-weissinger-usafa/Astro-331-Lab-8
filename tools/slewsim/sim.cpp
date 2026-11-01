// Host simulation of SLEW TEST / SLEW YAW on a pendulous air-bearing table, using the firmware's
// slew_test.cpp, attitude_ctrl.cpp, tune.cpp and wheel_geometry.h. Ideal wheel torques (no speed
// loop), perfect attitude/rate estimate. Usage: sim [yaw|fixed] [roomTorque_mNm] [trace.csv]
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
  const bool yawOnly = argc > 1 && !strcmp(argv[1], "yaw");
  const float room = (argc > 2 ? atof(argv[2]) : 2.0f) * 1e-3f;  // N m, fixed in the room
  FILE* trace = argc > 3 ? fopen(argv[3], "w") : nullptr;
  if (trace) fprintf(trace, "t_s,yaw_deg,yaw_ref_deg,roll_deg,pitch_deg,wA,wB,wC,wD,phase,target\n");
  const float I[3] = {0.14f, 0.14f, 0.0776f};
  tune::inertia[2] = 0.0776f;  // SET IZZ as on the table today (x/y left at the model value)
  const float m = 5.1f, g = 9.81f, L = 0.013f;
  const Vec3 rcg = {0.00004f, 0.00027f, -L};  // small horizontal offset => ~1 deg rest tilt
  const float J = params::kWheelInertia;

  Quat q = qFromEuler321(-1.2f * kDegToRad, -0.2f * kDegToRad, 0.3f);
  Vec3 w = {0, 0, 0};
  float ws[kN] = {0, 0, 0, 0};
  bool en[kN] = {true, true, true, true};

  SlewTest slew;
  if (!yawOnly)
    for (int i = 0; i < 4; i++) slew.setTarget(i, -1.2f, -0.2f, params::kSlewTargets[i][2]);
  AttitudeController ctrl;
  const float dt = 0.01f;
  // Let the free table settle first (no control), so the test starts from rest.
  for (int k = 0; k < 3000; k++) {
    const Mat<3, 3> A = qToMatrix(q);
    const Vec3 gb = matVec(A, {0, 0, -g * m});
    Vec3 tau = cross(rcg, gb) + matVec(A, {room, 0.5f * room, 0}) - w * 0.02f;  // heavy damping to settle
    const Vec3 Iw = {I[0] * w.x, I[1] * w.y, I[2] * w.z};
    tau = tau - cross(w, Iw);
    w = w + Vec3{tau.x / I[0], tau.y / I[1], tau.z / I[2]} * dt;
    q = qnormalize(qmul(qconj(qFromRotVec(w * dt)), q));
  }
  w = {0, 0, 0};
  const Vec3 e0 = qToEuler321(q);
  printf("start: roll %.2f pitch %.2f yaw %.1f deg, room torque %.1f mN m, mode %s\n", e0.x * kRadToDeg,
         e0.y * kRadToDeg, e0.z * kRadToDeg, room * 1e3f, yawOnly ? "SLEW YAW" : "SLEW TEST (fixed tilt)");

  slew.start(q, yawOnly);
  ctrl.setTrajectory(slew.refQ(), slew.refW(), slew.refA());
  float wmax = 0, tiltDevMax = 0;
  double t = 0;
  int doneAt = -1;
  for (int k = 0; k < 40000; k++, t += dt) {
    const SlewTest::Event ev = slew.step(dt, q);
    ctrl.setTrajectory(slew.refQ(), slew.refW(), slew.refA());
    Vec3 h = {0, 0, 0};
    for (int i = 0; i < kN; i++) h = h + axis(i) * (J * ws[i]);
    const Vec3 tauC = ctrl.update(q, w, dt, h);
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
      printf("%7.1f s  WHEEL LIMIT (%.0f rad/s) in target %d\n", t, wmax, slew.index() + 1);
      break;
    }
    if (doneAt >= 0 && k - doneAt > 6000) break;  // hold target 4 for 60 s
  }
  printf("result: %d/4 PASS, peak wheel %.1f rad/s, worst roll/pitch dev from rest in holds %.3f deg, t %.0f s\n",
         slew.passCount(), wmax, tiltDevMax, t);
  return 0;
}
