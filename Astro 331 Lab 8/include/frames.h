#pragma once

// Frame and sign conventions. Everything in the estimator and (later) the
// controller assumes these. Do not change one without auditing the other.
//
// BODY FRAME (fixed to the BNO085):
//   z points UP, x and y complete a right-handed set (x cross y = z).
//   x = roll axis, y = pitch axis, z = yaw axis.
//
// INERTIAL REFERENCE FRAME:
//   z points UP (opposite gravity), right-handed, x/y horizontal.
//   Yaw zero is defined at startup (power-on heading). There is NO magnetometer,
//   so heading is relative and yaw is dead-reckoned from the gyro.
//
// QUATERNION q = (w, x, y, z), Hamilton convention, scalar first, unit norm.
//   q takes INERTIAL vectors into the BODY frame:  v_body = q (x) v_inertial (x) q*
//
// ACCELEROMETER measures specific force. At rest with z up it reads +1 g on z,
// so the gravity reference direction (inertial) is +z = (0, 0, +1).
//
// GYRO: body-frame angular velocity of the body w.r.t. inertial, rad/s.
//
// EULER (display only), 3-2-1 sequence: yaw about z, pitch about y, roll about x.

constexpr float kGravity = 9.80665f;  // m/s^2, converts BNO085 accel to g
