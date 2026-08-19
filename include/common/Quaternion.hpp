#pragma once

#include <algorithm>
#include <cmath>

namespace autoaim::common {

struct Quaternion {
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};

  [[nodiscard]] double norm() const noexcept {
    return std::sqrt((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz));
  }

  void normalize() noexcept {
    const double n = norm();
    if (n <= 1e-12) {
      qw = 1.0;
      qx = qy = qz = 0.0;
      return;
    }
    qw /= n;
    qx /= n;
    qy /= n;
    qz /= n;
  }
};

inline Quaternion slerp(const Quaternion &a, const Quaternion &b,
                        double t) noexcept {
  t = std::clamp(t, 0.0, 1.0);

  Quaternion q1 = a;
  Quaternion q2 = b;
  q1.normalize();
  q2.normalize();
  double dot =
    (q1.qw * q2.qw) + (q1.qx * q2.qx) + (q1.qy * q2.qy) + (q1.qz * q2.qz);

  if (dot < 0.0) {
    dot = -dot;
    q2.qw = -q2.qw;
    q2.qx = -q2.qx;
    q2.qy = -q2.qy;
    q2.qz = -q2.qz;
  }

  if (dot > 0.9995) {
    Quaternion result{q1.qw + t * (q2.qw - q1.qw), q1.qx + t * (q2.qx - q1.qx),
                      q1.qy + t * (q2.qy - q1.qy), q1.qz + t * (q2.qz - q1.qz)};
    result.normalize();
    return result;
  }

  const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
  const double sin_theta = std::sin(theta);
  const double w1 = std::sin((1.0 - t) * theta) / sin_theta;
  const double w2 = std::sin(t * theta) / sin_theta;

  Quaternion result{w1 * q1.qw + w2 * q2.qw, w1 * q1.qx + w2 * q2.qx,
                    w1 * q1.qy + w2 * q2.qy, w1 * q1.qz + w2 * q2.qz};
  result.normalize();
  return result;
}

} // namespace autoaim::common
