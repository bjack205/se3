#pragma once

#include <concepts>
#include <ostream>

#ifdef QUAT_INIT_TO_IDENTITY
constexpr bool kInitializeQuatToIdentity = QUAT_INIT_TO_IDENTITY;
#else
constexpr bool kInitializeQuatToIdentity = true;
#endif

#include "se3/attitude/attitude_group.hpp"
#include "se3/linear_algebra/generic/matrices_generic.hpp"
#include "se3/linear_algebra/matmul.hpp"
#include "se3/linear_algebra/type_traits.hpp"
#include "se3/linear_algebra/vector_concepts.hpp"
#include "se3/linear_algebra/vector_ops.hpp"

namespace se3 {

template <typename Q, typename T = std::ranges::range_value_t<Q>>
concept AbstractQuaternion = Vec4<Q> and Q::IsQuaternion and
                             std::is_trivial_v<typename Q::MatrixGroup> and
                             requires(Q q, Q other, T val) {
                               { q.x } -> std::convertible_to<T>;
                               { q.y } -> std::convertible_to<T>;
                               { q.z } -> std::convertible_to<T>;
                               { q.w } -> std::convertible_to<T>;
                               { q == other } -> std::convertible_to<bool>;
                               { q != other } -> std::convertible_to<bool>;
                               {
                                 Q(val, val, val, val)
                               } -> std::convertible_to<Q>;
                             };

template <std::floating_point T>
constexpr T SmallAngleTolerance() {
  return 1e-7;
}

template <>
constexpr float SmallAngleTolerance<float>() {
  return 1e-5f;
}

template <AbstractQuaternion Q>
Q identity() {
  return {0, 0, 0, 1};
}

template <AbstractQuaternion Q>
Q randomRotation(std::uniform_random_bit_generator auto &gen) {
  using T = std::ranges::range_value_t<Q>;
  std::normal_distribution<T> dist(0.0, 1.0);
  return normalize(Q{dist(gen), dist(gen), dist(gen), dist(gen)});
}

template <AbstractQuaternion Q,
          std::floating_point T = std::ranges::range_value_t<Q>>
Q rotX(T angle) {
  T c = std::cos(angle / 2);
  T s = std::sin(angle / 2);
  return {s, 0, 0, c};
}

template <AbstractQuaternion Q,
          std::floating_point T = std::ranges::range_value_t<Q>>
Q rotY(T angle) {
  T c = std::cos(angle / 2);
  T s = std::sin(angle / 2);
  return {0, s, 0, c};
}

template <AbstractQuaternion Q,
          std::floating_point T = std::ranges::range_value_t<Q>>
Q rotZ(T angle) {
  T c = std::cos(angle / 2);
  T s = std::sin(angle / 2);
  return {0, 0, s, c};
}

//! Vector part of a quaternion
template <AbstractQuaternion Q>
Vec3TypeFor<Q> quatvec(const Q &q) {
  return {q.x, q.y, q.z};
}

template <AbstractQuaternion Q,
          std::floating_point T = std::ranges::range_value_t<Q>>
Q conjugate(const Q &q) {
  return {-q.x, -q.y, -q.z, q.w};
}

template <AbstractQuaternion Q,
          std::floating_point T = std::ranges::range_value_t<Q>>
Q inverse(const Q &q) {
  return conjugate(q) / normSquared(q);
}

// Quaternion multiplication
template <AbstractQuaternion Q,
          std::floating_point T = std::ranges::range_value_t<Q>>
Q operator*(const Q &q1, const Q &q2) {
  // q.w, -q.x, -q.y, -q.z,
  // q.x, q.w, -q.z, q.y,
  // q.y, q.z, q.w, -q.x,
  // q.z, -q.y, q.x, q.w
  T w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  T x = q1.x * q2.w + q1.w * q2.x - q1.z * q2.y + q1.y * q2.z;
  T y = q1.y * q2.w + q1.z * q2.x + q1.w * q2.y - q1.x * q2.z;
  T z = q1.z * q2.w - q1.y * q2.x + q1.x * q2.y + q1.w * q2.z;
  return {x, y, z, w};
}

template <AbstractQuaternion Q>
Mat3TypeFor<Q> rotationMatrix(const Q &q) {
  using T = std::ranges::range_value_t<Q>;
  T xx = q.x * q.x;
  T yy = q.y * q.y;
  T zz = q.z * q.z;
  T ww = q.w * q.w;
  T xy = q.x * q.y;
  T xz = q.x * q.z;
  T yz = q.y * q.z;
  T wx = q.w * q.x;
  T wy = q.w * q.y;
  T wz = q.w * q.z;
  // TODO: this depends on active/passive convention and handedness
  return {ww + xx - yy - zz, 2 * (xy - wz),     2 * (xz + wy),
          2 * (xy + wz),     ww - xx + yy - zz, 2 * (yz - wx),
          2 * (xz - wy),     2 * (yz + wx),     ww - xx - yy + zz};
}

// Rotation of a vector
template <AbstractQuaternion Q, AbstractVector3 V>
V operator*(const Q &q, const V &v) {
  // v_out = q x q(v) x q_c
  // v_out = R(q)^T L(q) H v
  // TODO: allow for special case of single-axis rotations
  return rotationMatrix(q) * v;
}

template <Vec3 V, AbstractQuaternion Q = QuatTypeFor<V>>
Q expPure(const V &v) {
  using T = std::ranges::range_value_t<V>;
  const T theta2 = normSquared(v);
  const T theta = std::sqrt(theta2);
  if (theta <= SmallAngleTolerance<T>()) {
    const T scale = 1 - theta2 / T(6);
    return normalize(
        Q{scale * v[0], scale * v[1], scale * v[2], 1 - theta2 * T(0.5)});
  }
  const T sin_theta = std::sin(theta);
  const T cos_theta = std::cos(theta);
  const T scale = sin_theta / theta;
  return normalize(Q{scale * v[0], scale * v[1], scale * v[2], cos_theta});
}

template <AbstractQuaternion Q>
Q exp(const Q &q) {
  Q exp_q_v = expPure(quatvec(q));
  auto exp_q_w = std::exp(q.w);
  return exp_q_v * exp_q_w;
}

template <Vec3 V, AbstractQuaternion Q = QuatTypeFor<V>>
Q expm(const V &v) {
  return expPure(v / 2);
}

template <AbstractQuaternion Q, Vec3 V = Vec3TypeFor<Q>>
V logUnit(const Q &q) {
  // NOTE: assumes q is a unit quaternion
  using T = std::ranges::range_value_t<Q>;
  V v = quatvec(q);
  T theta2 = normSquared(v);
  T theta = std::sqrt(theta2);
  T scale;
  if (theta <= SmallAngleTolerance<T>()) {
    scale = (1 - theta2 / (3 * q.w * q.w)) / q.w;
  } else {
    scale = std::atan2(theta, q.w) / theta;
  }
  return {v[0] * scale, v[1] * scale, v[2] * scale};
}

template <AbstractQuaternion Q, Vec3 V = Vec3TypeFor<Q>>
Q log(const Q &q) {
  auto q_norm = norm(q);
  V log_q_unit = logUnit(q / q_norm);
  auto log_q_norm = std::log(q_norm);
  return {log_q_unit[0], log_q_unit[1], log_q_unit[2], log_q_norm};
}

template <AbstractQuaternion Q, Vec3 V = Vec3TypeFor<Q>>
V logm(const Q &q) {
  return 2 * logUnit(normalize(q));
}

// Computes the angle (in radians) between two unit quaternions
// The result is always in [0, pi]
template <AbstractQuaternion Q>
auto angleBetween(const Q &q1, const Q &q2) {
  using T = std::ranges::range_value_t<Q>;

  T diff = norm(q1 - q2);
  if (diff < SmallAngleTolerance<T>()) {
    return diff;
  }

  // Compute the dot product
  T d = dot(normalize(q1), normalize(q2));

  // Clamp to [-1, 1] to avoid domain errors in acos
  d = std::clamp(d, T(-1), T(1));

  // Use absolute value on the dot product to keep the comparison in the same
  // hemisphere.
  T angle = 2 * std::acos(std::abs(d));
  return angle;
}

template <AbstractQuaternion Q>
Q flipQuaternion(const Q &q) {
  return {-q.x, -q.y, -q.z, -q.w};
}

template <AbstractQuaternion Q>
Q flipToPositiveScalar(const Q &q) {
  using T = std::ranges::range_value_t<Q>;
  if (std::abs(q.w) < SmallAngleTolerance<T>()) {
    return q.x > 0 ? q : flipQuaternion(q);
  }
  return q.w > 0 ? q : flipQuaternion(q);
}

namespace quatmats {
template <AbstractQuaternion Q, Mat4 M = Mat4TypeFor<Q>>
M L(const Q &q) {
  // clang-format off
  return {
    q.w, -q.z, q.y, q.x, 
    q.z, q.w, -q.x, q.y, 
    -q.y, q.x, q.w, q.z, 
    -q.x, -q.y, -q.z, q.w 
  };
  // clang-format on
}

template <AbstractQuaternion Q, Mat4 M = Mat4TypeFor<Q>>
M R(const Q &q) {
  // clang-format off
  return {
    q.w, +q.z, -q.y,  q.x, 
    -q.z, q.w, +q.x,  q.y, 
    +q.y, -q.x, q.w,  q.z,
    -q.x, -q.y, -q.z, q.w 
  };
  // clang-format on
}

// TODO: this should be a Diag4
template <Mat4 M = generic::Matrix4<double>>
M T() {
  // clang-format off
  return {
    -1, 0, 0, 0, 
    0, -1, 0, 0, 
    0, 0, -1, 0,
    0, 0, 0, 1,
  };
  // clang-format on
}
}  // namespace quatmats

}  // namespace se3