#pragma once
#include <math.h>

// Minimal fixed-size linear algebra and quaternion helpers (float, no heap).
// Conventions are documented in frames.h.

struct Vec3 {
  float x, y, z;
};

struct Quat {
  float w, x, y, z;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float norm(const Vec3& a) { return sqrtf(dot(a, a)); }

template <int R, int C>
struct Mat {
  float a[R][C];
  Mat() {
    for (int i = 0; i < R; i++)
      for (int j = 0; j < C; j++) a[i][j] = 0.0f;
  }
  static Mat identity() {
    Mat m;
    for (int i = 0; i < (R < C ? R : C); i++) m.a[i][i] = 1.0f;
    return m;
  }
  float& operator()(int i, int j) { return a[i][j]; }
  const float& operator()(int i, int j) const { return a[i][j]; }
};

template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& A, const Mat<K, C>& B) {
  Mat<R, C> out;
  for (int i = 0; i < R; i++)
    for (int k = 0; k < K; k++) {
      const float aik = A(i, k);
      for (int j = 0; j < C; j++) out(i, j) += aik * B(k, j);
    }
  return out;
}

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& A, const Mat<R, C>& B) {
  Mat<R, C> out;
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++) out(i, j) = A(i, j) + B(i, j);
  return out;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& A, const Mat<R, C>& B) {
  Mat<R, C> out;
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++) out(i, j) = A(i, j) - B(i, j);
  return out;
}

template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& A) {
  Mat<C, R> out;
  for (int i = 0; i < R; i++)
    for (int j = 0; j < C; j++) out(j, i) = A(i, j);
  return out;
}

// Inverse of a 3x3 via the adjugate. Returns false if (near) singular.
inline bool inverse3(const Mat<3, 3>& m, Mat<3, 3>& inv) {
  const float c00 = m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1);
  const float c01 = m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2);
  const float c02 = m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0);
  const float det = m(0, 0) * c00 + m(0, 1) * c01 + m(0, 2) * c02;
  if (fabsf(det) < 1e-20f) return false;
  const float id = 1.0f / det;
  inv(0, 0) = c00 * id;
  inv(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * id;
  inv(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * id;
  inv(1, 0) = c01 * id;
  inv(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * id;
  inv(1, 2) = (m(0, 2) * m(1, 0) - m(0, 0) * m(1, 2)) * id;
  inv(2, 0) = c02 * id;
  inv(2, 1) = (m(0, 1) * m(2, 0) - m(0, 0) * m(2, 1)) * id;
  inv(2, 2) = (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0)) * id;
  return true;
}

// Cross-product (skew-symmetric) matrix: skew(a) * b = a x b
inline Mat<3, 3> skew(const Vec3& v) {
  Mat<3, 3> m;
  m(0, 1) = -v.z;  m(0, 2) = v.y;
  m(1, 0) = v.z;   m(1, 2) = -v.x;
  m(2, 0) = -v.y;  m(2, 1) = v.x;
  return m;
}

// ---- Quaternions (Hamilton, scalar first) ----

inline Quat qmul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quat qconj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Quat qnormalize(const Quat& q) {
  const float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-12f) return {1.0f, 0.0f, 0.0f, 0.0f};
  const float s = 1.0f / n;
  return {q.w * s, q.x * s, q.y * s, q.z * s};
}

// Quaternion for an ACTIVE rotation by rotation vector r (axis * angle, rad).
inline Quat qFromRotVec(const Vec3& r) {
  const float ang = norm(r);
  if (ang < 1e-8f) return qnormalize({1.0f, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z});
  const float s = sinf(0.5f * ang) / ang;
  return {cosf(0.5f * ang), r.x * s, r.y * s, r.z * s};
}

// Rotation matrix A(q) with v_body = A * v_inertial (q is inertial -> body).
inline Mat<3, 3> qToMatrix(const Quat& q) {
  const float w = q.w, x = q.x, y = q.y, z = q.z;
  Mat<3, 3> m;
  m(0, 0) = 1.0f - 2.0f * (y * y + z * z);
  m(0, 1) = 2.0f * (x * y - w * z);
  m(0, 2) = 2.0f * (x * z + w * y);
  m(1, 0) = 2.0f * (x * y + w * z);
  m(1, 1) = 1.0f - 2.0f * (x * x + z * z);
  m(1, 2) = 2.0f * (y * z - w * x);
  m(2, 0) = 2.0f * (x * z - w * y);
  m(2, 1) = 2.0f * (y * z + w * x);
  m(2, 2) = 1.0f - 2.0f * (x * x + y * y);
  return m;
}

// Euler angles (roll, pitch, yaw) in rad for the body attitude, 3-2-1 sequence.
// Returned as Vec3{roll(x), pitch(y), yaw(z)}.
inline Vec3 qToEuler321(const Quat& q_ib) {
  const Quat p = qconj(q_ib);  // body -> inertial
  Vec3 e;
  e.x = atan2f(2.0f * (p.w * p.x + p.y * p.z), 1.0f - 2.0f * (p.x * p.x + p.y * p.y));
  float s = 2.0f * (p.w * p.y - p.z * p.x);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  e.y = asinf(s);
  e.z = atan2f(2.0f * (p.w * p.z + p.x * p.y), 1.0f - 2.0f * (p.y * p.y + p.z * p.z));
  return e;
}

constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;
