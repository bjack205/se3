//
// Created by Brian Jackson on 7/9/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once
#include <array>

#include "se3/linear_algebra/vectors.hpp"

namespace se3 {

struct ArrayGroup;

namespace arrays {

template <std::floating_point T>
struct Vector3 {
  static constexpr std::size_t SizeAtCompileTime = 3;
  using Scalar = T;
  using MatrixGroup = ArrayGroup;

  Vector3() = default;
  Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

  T& operator[](int i) { return v[i]; }
  const T& operator[](int i) const { return v[i]; }

  T* data() { return v.data(); }
  const T* data() const { return v.data(); }

  T* begin() { return v.data(); }
  T* end() { return v.data() + SizeAtCompileTime; }
  const T* begin() const { return v.data(); }
  const T* end() const { return v.data() + SizeAtCompileTime; }

  bool operator==(const Vector3& other) const { return v == other.v; }

  union {
  std::array<T, 3> v;
    struct {
      T x, y, z;
    };
  };
};



}

template <std::floating_point T>
auto normSquared(const arrays::Vector3<T> &x) {
  T n = 0;
  for (int i = 0; i < 3; ++i) {
    n += x[i] * x[i];
  }
  return n;
}
}
