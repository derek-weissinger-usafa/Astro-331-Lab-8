// Offline replay of an SD log through the firmware's own estimator and magnetometer aid.
//
// Builds attitude_ekf.cpp and mag_aid.cpp from the PlatformIO project unchanged (see build.sh),
// feeds them the logged raw gyro / accelerometer / magnetometer / wheel data row by row, and
// writes the replayed yaw next to the logged one. With the firmware's settings the two should
// agree closely (that checks the replay); then options change one thing at a time to see what it
// would have done to the same run:
//   --sigma <rad>   heading noise per mag sample (firmware kMagHeadingSigmaRad)
//   --nomag         gyro-only yaw (MAG OFF)
//   --predict / --learn  new direction patterns: predict the reference from a known one (the
//                   firmware default since 2026-10-31, kMagPredictNewPatterns) or learn it in place
//                   (the behaviour MAGTEST_0001 ran with)
//   --nokick        don't copy KICKs from the log (for a gyro-only reference line)
//   --hard x,y,z    table hard iron to remove, uT (tools/hardiron_fit.py; firmware SET MAG_HX/HY/HZ)
//   --room c1,s1,c2,s2  room distortion correction, deg (tools/mag_room_fit.py; SET MAG_C1/S1/C2/S2)
//   --gyro-noise v / --accel-sigma v / --bias-walk v / --bias-walk-z v   EkfParams (attitude_ekf.h;
//                   still the original placeholders) - try values on recorded runs
//
// Usage: replay <LOG.CSV> <out.csv> [options]
// The log must contain the rows from a calibration end (state C -> I) or start after one (the
// first row's logged attitude and gyro bias seed the filter). Rows: one per gyro sample. A logged
// yaw jump > 3 deg in one row is a KICK and is applied to the replay too. Encoder counts come
// from the enc_a..d columns (logs from 2026-10-30 on) or are rebuilt from the wheel speeds.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "attitude_ekf.h"
#include "control_params.h"
#include "mag_aid.h"

static std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  size_t a = 0;
  for (;;) {
    const size_t b = s.find(',', a);
    out.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
    if (b == std::string::npos) break;
    a = b + 1;
  }
  return out;
}

static float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: replay <LOG.CSV> <out.csv> [--sigma rad] [--nomag] [--predict]\n");
    return 1;
  }
  float sigma = params::kMagHeadingSigmaRad;
  bool noMag = false, predict = params::kMagPredictNewPatterns, noKick = false;
  Vec3 hard{params::kMagHardIronUt[0], params::kMagHardIronUt[1], params::kMagHardIronUt[2]};
  float room[4] = {params::kMagRoomCorrDeg[0], params::kMagRoomCorrDeg[1], params::kMagRoomCorrDeg[2],
                   params::kMagRoomCorrDeg[3]};
  EkfParams ep;
  bool epChanged = false;
  for (int i = 3; i < argc; i++) {
    if (!strcmp(argv[i], "--sigma") && i + 1 < argc) sigma = (float)atof(argv[++i]);
    else if (!strcmp(argv[i], "--hard") && i + 1 < argc && sscanf(argv[++i], "%f,%f,%f", &hard.x, &hard.y, &hard.z) == 3) {}
    else if (!strcmp(argv[i], "--room") && i + 1 < argc &&
             sscanf(argv[++i], "%f,%f,%f,%f", &room[0], &room[1], &room[2], &room[3]) == 4) {}
    else if (!strcmp(argv[i], "--gyro-noise") && i + 1 < argc) { ep.gyroNoise = (float)atof(argv[++i]); epChanged = true; }
    else if (!strcmp(argv[i], "--accel-sigma") && i + 1 < argc) { ep.accelSigma = (float)atof(argv[++i]); epChanged = true; }
    else if (!strcmp(argv[i], "--bias-walk") && i + 1 < argc) { ep.gyroBiasWalk = (float)atof(argv[++i]); epChanged = true; }
    else if (!strcmp(argv[i], "--bias-walk-z") && i + 1 < argc) { ep.gyroBiasWalkZ = (float)atof(argv[++i]); epChanged = true; }
    else if (!strcmp(argv[i], "--nomag")) noMag = true;
    else if (!strcmp(argv[i], "--predict")) predict = true;
    else if (!strcmp(argv[i], "--learn")) predict = false;
    else if (!strcmp(argv[i], "--nokick")) noKick = true;
  }
  FILE* in = fopen(argv[1], "r");
  FILE* out = fopen(argv[2], "w");
  if (!in || !out) {
    fprintf(stderr, "cannot open files\n");
    return 1;
  }

  static char line[2048];
  std::map<std::string, int> col;
  if (!fgets(line, sizeof(line), in)) return 1;
  {
    std::string h(line);
    while (!h.empty() && (h.back() == '\n' || h.back() == '\r')) h.pop_back();
    const auto names = split(h);
    for (size_t i = 0; i < names.size(); i++) col[names[i]] = (int)i;
  }
  auto need = [&](const char* n) {
    if (!col.count(n)) {
      fprintf(stderr, "log has no column %s\n", n);
      exit(1);
    }
    return col[n];
  };
  const int cT = need("mcutime_ms"), cG = need("gyro_x_rad_s"), cA = need("accel_x_m_s2"), cM = need("mag_x_uT");
  const int cW = need("motor_a_measured_rate_rpm"), cYaw = need("yaw_deg"), cBias = need("bias_x_rad_s");
  const int cState = need("state");
  // optional (older logs): test_step (MAG MAP / MAG TEST / STEP TEST), mag_mode (aid, 2026-10-27 on)
  const int cStep = col.count("test_step") ? col["test_step"] : -1;
  const int cMode = col.count("mag_mode") ? col["mag_mode"] : -1;
  const int cLast = cState > cYaw ? cState : cYaw;
  const int cWs = col.count("wheel_status") ? col["wheel_status"] : -1;
  const int cEnc = col.count("enc_a") ? col["enc_a"] : -1;

  AttitudeEKF ekf(ep);
  MagAid aid;
  aid.setEnabled(!noMag);
  aid.setPredictAlways(predict);
  aid.setHardIron(hard);
  aid.setRoomCorrection(room);
  fprintf(out, "t_s,test_step,state,yaw_log_deg,yaw_replay_deg,mode_log,mode_replay,resid_replay_deg,bias_z_replay_deg_s\n");

  bool init = false;
  char prevState = '?';
  double tPrev = 0, tMag = 0;
  float yawLogPrev = 0;
  Vec3 aPrev{0, 0, 0}, mPrev{0, 0, 0};
  double cntF[4] = {0, 0, 0, 0};
  long rows = 0, used = 0, inUse = 0, kicks = 0;
  double sumD2 = 0, maxD = 0;
  const double kRadPerCount = 6.283185307 / params::kEncoderCountsPerRev;

  while (fgets(line, sizeof(line), in)) {
    std::string s(line);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    if (s.empty() || !(isdigit((unsigned char)s[0]))) continue;  // repeated headers, messages
    const auto f = split(s);
    if ((int)f.size() <= cLast || (int)f.size() <= cMode) continue;
    rows++;
    const double t = atof(f[cT].c_str()) * 1e-3;
    // The log rounds the BNO085's fixed-point gyro (Q9, 1/512 rad/s) to 5 decimals (-1 LSB =
    // -0.001953 is logged as -0.00195): snap it back to the sensor's steps.
    auto q = [&](int c, double lsb) { return (float)(lsb * std::lround(atof(f[c].c_str()) / lsb)); };
    const double kG = 1.0 / 512;
    const Vec3 g{q(cG, kG), q(cG + 1, kG), q(cG + 2, kG)};
    const Vec3 a{(float)atof(f[cA].c_str()), (float)atof(f[cA + 1].c_str()), (float)atof(f[cA + 2].c_str())};
    const Vec3 m{(float)atof(f[cM].c_str()), (float)atof(f[cM + 1].c_str()), (float)atof(f[cM + 2].c_str())};
    float spd[4];
    for (int i = 0; i < 4; i++) spd[i] = (float)(atof(f[cW + i].c_str()) * 6.283185307 / 60.0);
    const char state = f[cState].empty() ? '?' : f[cState][0];
    const float yawLog = (float)atof(f[cYaw].c_str());
    const int step = cStep >= 0 ? atoi(f[cStep].c_str()) : -1;
    const char modeLog = cMode < 0 || f[cMode].empty() ? '-' : f[cMode][0];

    // Seed at the first row after a calibration (or at the first row if the log starts after one).
    if (state != 'C' && (!init || prevState == 'C')) {
      const Vec3 b{(float)atof(f[cBias].c_str()), (float)atof(f[cBias + 1].c_str()), (float)atof(f[cBias + 2].c_str())};
      ekf.initialize(a, b, 1.5e-5f);
      ekf.perturbYaw(-yawLog * 0.01745329f);  // perturbYaw(+x) lowers the displayed yaw by x
      aid.reset();
      init = true;
      tPrev = tMag = t;
      aPrev = a;
      mPrev = m;
      yawLogPrev = yawLog;
      prevState = state;
      continue;
    }
    prevState = state;
    if (!init || state == 'C') continue;

    const double dt = t - tPrev;
    tPrev = t;
    // KICK: the logged yaw jumped in one row (the table can't turn that fast)
    const float jump = wrap180(yawLog - yawLogPrev);
    yawLogPrev = yawLog;
    if (fabsf(jump) > 3.0f && fabsf(g.z) < 0.5f) {
      if (!noKick) ekf.perturbYaw(-jump * 0.01745329f);
      kicks++;
    }
    if (dt > 0 && dt <= params::kImuTimeoutMs * 1e-3) ekf.predict(g, (float)dt);
    for (int i = 0; i < 4; i++) cntF[i] += (dt > 0 && dt < 1.0 ? spd[i] * dt : 0.0) / kRadPerCount;

    if (a.x != aPrev.x || a.y != aPrev.y || a.z != aPrev.z) {
      ekf.updateAccel(a);
      aPrev = a;
    }
    if (m.x != mPrev.x || m.y != mPrev.y || m.z != mPrev.z) {
      const float dtMag = (float)(t - tMag);
      tMag = t;
      mPrev = m;
      long cnt[4];
      bool wheelsOn[4];
      for (int i = 0; i < 4; i++) {
        cnt[i] = cEnc >= 0 ? atol(f[cEnc + i].c_str()) : (long)floor(cntF[i]);
        wheelsOn[i] = cWs < 0 || (int)f[cWs].size() <= i || f[cWs][i] != '-';
      }
      Vec3 mUse, ref;
      if (dtMag > 0.0f && dtMag < 0.1f && aid.update(m, cnt, spd, wheelsOn, ekf.quaternion(), dtMag, mUse, ref))
        ekf.updateMagYaw(mUse, ref, sigma);
    }

    const float yawRep = ekf.eulerRad().z * 57.29578f;
    const double d = wrap180(yawRep - yawLog);
    sumD2 += d * d;
    if (fabs(d) > maxD) maxD = fabs(d);
    used++;
    const char mr = aid.modeChar();
    if (mr == 's' || mr == 'S') inUse++;
    if (used % 10 == 0)
      fprintf(out, "%.3f,%d,%c,%.3f,%.3f,%c,%c,%.3f,%.6f\n", t, step, state, yawLog, yawRep, modeLog, mr,
              aid.headingResidualDeg(), ekf.gyroBias().z * 57.29578f);
  }
  fclose(out);
  printf("rows %ld replayed %ld, kicks %ld, aid in use %.1f%%, predicted refs %d\n", rows, used, kicks,
         used ? 100.0 * inUse / used : 0.0, aid.predictedCount());
  printf("replay - log yaw: rms %.3f deg, max %.3f deg%s\n", used ? sqrt(sumD2 / used) : 0.0, maxD,
         (noMag || predict != params::kMagPredictNewPatterns || epChanged || sigma != params::kMagHeadingSigmaRad) ? "  (settings differ from the firmware)" : "");
  return 0;
}
