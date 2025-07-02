#pragma once

#include "se3/linear_algebra/vectors.hpp"

namespace se3 {

// Generic matrix group
struct Generic;

namespace generic {

template <std::floating_point T>
struct Vector3 {
  static constexpr std::size_t SizeAtCompileTime = 3;
  using Scalar = T;
  using MatrixGroup = Generic;

  Vector3() : x(0), y(0), z(0) {}
  Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
  Vector3(const AbstractVector3 auto& v) : x(v[0]), y(v[1]), z(v[2]) {}

  static Vector3 Zero() { return {0, 0, 0}; }
  static Vector3 UnitX() { return {1, 0, 0}; }
  static Vector3 UnitY() { return {0, 1, 0}; }
  static Vector3 UnitZ() { return {0, 0, 1}; }

  T& operator[](int i) { return (&x)[i]; }
  const T& operator[](int i) const { return (&x)[i]; }

  T* data() { return &x; }
  const T* data() const { return &x; }

  T* begin() { return &x; }
  T* end() { return &x + SizeAtCompileTime; }
  const T* begin() const { return &x; }
  const T* end() const { return &x + SizeAtCompileTime; }

  auto operator<=>(const Vector3& other) const = default;

  T x, y, z;
};

template <std::floating_point T>
struct Vector4 {
  static constexpr std::size_t SizeAtCompileTime = 4;
  using Scalar = T;
  using MatrixGroup = Generic;

  T& operator[](int i) { return (&x)[i]; }
  const T& operator[](int i) const { return (&x)[i]; }

  T* data() { return &x; }
  const T* data() const { return &x; }

  T* begin() { return &x; }
  T* end() { return &x + SizeAtCompileTime; }
  const T* begin() const { return &x; }
  const T* end() const { return &x + SizeAtCompileTime; }

  auto operator<=>(const Vector4& other) const = default;

  T x, y, z, w;
};

}  // namespace generic
}  // namespace se3