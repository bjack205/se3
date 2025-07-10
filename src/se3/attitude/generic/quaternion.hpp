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
struct Quaternion {
  static constexpr std::size_t SizeAtCompileTime = 4;
  using MatrixGroup = Generic;

  Quaternion() = default;
  Quaternion(T q0, T q1, T q2, T q3) : w{q0}, x{q1}, y{q2}, z{q3} {}

  explicit Quaternion(const AbstractVector4 auto &vec4)
      : Quaternion(vec4[0], vec4[1], vec4[2], vec4[3]) {}

  static Quaternion Pure(const AbstractVector3 auto &vec3) {
    return Quaternion(0, vec3[0], vec3[1], vec3[2]);
  }

  static Quaternion Identity() { return Quaternion(1, 0, 0, 0); }

  const T *data() const { return &w; }
  T *data() { return &w; }
  [[nodiscard]] std::size_t size() const { return 4; }

  struct iterator {
    int8_t index = 0;
    Quaternion *parent;
    const T *operator*() const { return parent->operator[](index); }
  };

  T *begin() { return &w; }
  T *end() { return &z + 1; }
  const T *begin() const { return &w; }
  const T *end() const { return &z + 1; }

  const T &operator[](int i) const { return (&w)[i]; }
  T &operator[](int i) { return (&w)[i]; }

  auto operator<=>(const Quaternion &other) const = default;

  T w = static_cast<T>(kInitializeQuatToIdentity);
  T x = 0;
  T y = 0;
  T z = 0;
};

}  // namespace generic

}  // namespace se3