//
// Created by Brian Jackson on 7/9/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include "se3/attitude/quaternion.hpp"
#include "se3/linear_algebra/type_traits.hpp"
#include "se3/linear_algebra/vector_concepts.hpp"

namespace se3 {

namespace generic {

template <std::floating_point T>
struct Quaternion : QuaternionBase {
  static constexpr std::size_t SizeAtCompileTime = 4;
  using MatrixGroup = Generic;

  Quaternion() = default;
  Quaternion(T q0, T q1, T q2, T q3) : x{q0}, y{q1}, z{q2}, w{q3} {}

  explicit Quaternion(const AbstractVector4 auto &vec4)
      : Quaternion(vec4[0], vec4[1], vec4[2], vec4[3]) {}

  static Quaternion Pure(const AbstractVector3 auto &vec3) {
    return Quaternion(vec3[0], vec3[1], vec3[2], 0);
  }

  const T *data() const { return &w; }
  T *data() { return &w; }
  [[nodiscard]] std::size_t size() const { return 4; }

  T *begin() { return &x; }
  T *end() { return &w + 1; }
  const T *begin() const { return &x; }
  const T *end() const { return &w + 1; }

  const T &operator[](int i) const { return (&x)[i]; }
  T &operator[](int i) { return (&x)[i]; }

  bool operator==(const Quaternion &other) const {
    return x == other.x && y == other.y && z == other.z && w == other.w;
  }

  T x = 0;
  T y = 0;
  T z = 0;
  T w = static_cast<T>(kInitializeQuatToIdentity);
};

}  // namespace generic

}  // namespace se3