//
// Created by Brian Jackson on 6/16/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include <Eigen/Dense>

#include "quaternion.hpp"

namespace se3 {

template <Vec3 V, std::floating_point T = std::ranges::range_value_t<V>>
V operator*(const Quaternion<T> &q, const V &v) {
  // v_out = q x q(v) x q_c
  // v_out = R(q)^T L(q) H v
  const auto [vx, vy, vz] = v;
  T w = -(q.x() * vx + q.y() * vy + q.z() * vz);
  T x = +q.w() - q.y() * vz + q.z() * vy;
  T y = +q.x() * vz + q.w() - q.z() * vx;
  T z = -q.x() * vy + v.y() * vx + q.w();
}

template <std::floating_point T> Eigen::Matrix4<T> L(const Quaternion<T> &q) {
  Eigen::Matrix4<T> L;
  // clang-format off
  L << +q.w, -q.x, -q.y, -q.z,
       +q.x, +q.w, -q.z, +q.y,
       +q.y, +q.z, +q.w, -q.x,
       +q.z, -q.y, +q.x, +q.w;
  // clang-format on
}

template <std::floating_point T> Eigen::Matrix4<T> R(const Quaternion<T> &q) {
  Eigen::Matrix4<T> L;
  // clang-format off
  L << +q.w, -q.x, -q.y, -q.z,
       +q.x, +q.w, +q.z, -q.y,
       +q.y, -q.z, +q.w, +q.x,
       +q.z, +q.y, -q.x, +q.w;
  // clang-format on
}

template <std::floating_point T>
Quaternion<T> operator*(const Quaternion<T> &q0, const Quaternion<T> &q1) {
  Eigen::Vector4<T> row;
  Eigen::Vector4<T> col(q1.w, q1.x, q1.y, q1.z);
  row << q0.w, q0.x, q0.y, q0.z;
  const T w = row.dot(col);
  row << q0.x, q0.w, q0.z, -q0.y;
  const T x = row.dot(col);
  row << q1.y, +q0.z, +q0.w, -q0.x;
  const T y = row.dot(col);
  row << q1.z, -q0.y, +q0.x, +q0.w;
  const T z = row.dot(col);
  return {w, x, y, z};
}

template <Vec3 V, std::floating_point T = std::ranges::range_value_t<V>>
Quaternion<T> pure_quat_expm(const V &v) {
  constexpr auto kSmallAngleTolerance = 1e-6;
  const T theta2 = normSquared(v);
  const T theta = std::sqrt(theta2);
  if (theta <= kSmallAngleTolerance) {
    const T scale = 1 - theta2 / T(6);
    return {1 - theta2 * 0.5, scale * v[0], scale * v[1], scale * v[2]};
  }
  const T sin_theta = std::sin(theta);
  const T cos_theta = std::cos(theta);
  const T scale = sin_theta / theta;
  return {cos_theta, scale * v[0], scale * v[1], scale * v[2]};
};
}
}