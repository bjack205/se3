#pragma once

#include <random>

#include "vector_concepts.hpp"
#include "vectors.hpp"

namespace se3 {

// TODO: Make Unit vectors special types

template <Vec3 V, int N>
  requires(N >= 0 && N < 3)
V Unit() {
  if constexpr (N == 0) {
    return {1, 0, 0};
  } else if constexpr (N == 1) {
    return {0, 1, 0};
  } else if constexpr (N == 2) {
    return {0, 0, 1};
  } else {
    static_assert(N >= 0 && N < 3, "Index out of bounds for Unit vector");
  }
}

template <Vec4 V, int N>
  requires(N >= 0 && N < 4)
V Unit() {
  if constexpr (N == 0) {
    return {1, 0, 0, 0};
  } else if constexpr (N == 1) {
    return {0, 1, 0, 0};
  } else if constexpr (N == 2) {
    return {0, 0, 1, 0};
  } else if constexpr (N == 3) {
    return {0, 0, 0, 1};
  } else {
    static_assert(N >= 0 && N < 4, "Index out of bounds for Unit vector");
  }
}

// Initialization
template <Vec3or4 V>
V UnitX() {
  return Unit<V, 0>();
}

template <Vec3or4 V>
V UnitY() {
  return Unit<V, 1>();
}

template <Vec3or4 V>
V UnitZ() {
  return Unit<V, 2>();
}

template <Vec4 V>
V UnitW() {
  return Unit<V, 3>();
}

template <Vec3 V>
V Zero() {
  return {0.0, 0.0, 0.0};
}

template <Vec3 V>
V Ones() {
  return {1.0, 1.0, 1.0};
}

template <Vec3 V, std::floating_point T>
  requires(std::same_as<T, std::ranges::range_value_t<V>>)
V Constant(T val) {
  return {val, val, val};
}

template <Vec3 V, std::floating_point T>
  requires(std::same_as<T, std::ranges::range_value_t<V>>)
V Sequence(T start, T step = T(1)) {
  return {start, start + step, start + T(2) * step};
}

// Setters
template <Vec3 V, std::floating_point T = std::ranges::range_value_t<V>>
void setValues(V& v, const std::tuple<T, T, T>& values) {
  v.x = std::get<0>(values);
  v.y = std::get<1>(values);
  v.z = std::get<2>(values);
}

template <Vec3 V>
void setUnitX(V& v) {
  setValues(v, {1, 0, 0});
}

template <Vec3 V>
void setUnitY(V& v) {
  setValues(v, {0, 1, 0});
}

template <Vec3 V>
void setUnitZ(V& v) {
  setValues(v, {0, 0, 1});
}

template <Vec3 V>
void setZero(V& v) {
  setValues(v, {0, 0, 0});
}

template <Vec3 V>
void setOnes(V& v) {
  setValues(v, {1, 1, 1});
}

template <Vec3 V, std::floating_point T>
void setConstant(V& v, T val) {
  setValues(v, {val, val, val});
}

template <Vec3or4 V, int N = SizeAtCompileTime<V>()>
V rand(std::uniform_random_bit_generator auto& gen) {
  using T = std::ranges::range_value_t<V>;
  std::uniform_real_distribution<T> dist(0.0, 1.0);
  if constexpr (N == 3) {
    return {dist(gen), dist(gen), dist(gen)};
  }
  if constexpr (N == 4) {
    return {dist(gen), dist(gen), dist(gen), dist(gen)};
  }
}

template <Vec3or4 V, int N = SizeAtCompileTime<V>()>
V randn(std::uniform_random_bit_generator auto& gen) {
  using T = std::ranges::range_value_t<V>;
  std::normal_distribution<T> dist(0.0, 1.0);
  if constexpr (N == 3) {
    return {dist(gen), dist(gen), dist(gen)};
  }
  if constexpr (N == 4) {
    return {dist(gen), dist(gen), dist(gen), dist(gen)};
  }
}

// Compound assignment operators for Vec3
template <Vec3 V>
V& operator+=(V& a, const V& b) {
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}

template <Vec3 V>
V& operator-=(V& a, const V& b) {
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;
  return a;
}

template <Vec3 V>
V& operator*=(V& a, const std::ranges::range_value_t<V>& c) {
  a.x *= c;
  a.y *= c;
  a.z *= c;
  return a;
}

template <Vec3 V>
V& operator/=(V& a, const std::ranges::range_value_t<V>& c) {
  a.x /= c;
  a.y /= c;
  a.z /= c;
  return a;
}

template <Vec4 V, std::floating_point T = std::ranges::range_value_t<V>>
void setValues(V& v, const std::tuple<T, T, T, T>& values) {
  v.x = std::get<0>(values);
  v.y = std::get<1>(values);
  v.z = std::get<2>(values);
  v.w = std::get<3>(values);
}

template <Vec4 V, std::floating_point T>
void setConstant(V& v, T val) {
  setValues(v, {val, val, val, val});
}

// Compound assignment operators for Vec4
template <Vec4 V>
V& operator+=(V& a, const V& b) {
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  a.w += b.w;
  return a;
}

template <Vec4 V>
V& operator-=(V& a, const V& b) {
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;
  a.w -= b.w;
  return a;
}

template <Vec4 V>
V& operator*=(V& a, const std::ranges::range_value_t<V>& c) {
  a.x *= c;
  a.y *= c;
  a.z *= c;
  a.w *= c;
  return a;
}

template <Vec4 V>
V& operator/=(V& a, const std::ranges::range_value_t<V>& c) {
  a.x /= c;
  a.y /= c;
  a.z /= c;
  a.w /= c;
  return a;
}

// Vector addition
template <Vec3 V>
V operator+(const V& a, const V& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

template <Vec4 V>
V operator+(const V& a, const V& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

// Vector subtraction
template <Vec3 V>
V operator-(const V& a, const V& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

template <Vec4 V>
V operator-(const V& a, const V& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}

// Scalar multiplication (right)
template <Vec3 V>
V operator*(const V& a, const std::ranges::range_value_t<V>& c) {
  return {a.x * c, a.y * c, a.z * c};
}

template <Vec4 V>
V operator*(const V& a, const std::ranges::range_value_t<V>& c) {
  return {a.x * c, a.y * c, a.z * c, a.w * c};
}

// Scalar multiplication (left)
template <Vec3 V>
V operator*(const std::ranges::range_value_t<V>& c, const V& a) {
  return {c * a.x, c * a.y, c * a.z};
}

template <Vec4 V>
V operator*(const std::ranges::range_value_t<V>& c, const V& a) {
  return {c * a.x, c * a.y, c * a.z, c * a.w};
}

// Negation
template <Vec3 V>
V operator-(const V& a) {
  return {-a.x, -a.y, -a.z};
}

template <Vec4 V>
V operator-(const V& a) {
  return {-a.x, -a.y, -a.z, -a.w};
}

// Scalar division
template <Vec3 V>
V operator/(const V& a, const std::ranges::range_value_t<V>& c) {
  return {a.x / c, a.y / c, a.z / c};
}

template <Vec4 V>
V operator/(const V& a, const std::ranges::range_value_t<V>& c) {
  return {a.x / c, a.y / c, a.z / c, a.w / c};
}

// Sum
template <AbstractFixedSizeVector V,
          std::floating_point T = std::ranges::range_value_t<V>>
T sum(const V& a) {
  constexpr auto N = SizeAtCompileTime<V>();
  T n = 0;
  for (int i = 0; i < N; ++i) {
    n += a[i];
  }
  return n;
}

// Norm
template <AbstractFixedSizeVector V>
auto normSquared(const V& a) {
  constexpr auto N = SizeAtCompileTime<V>();
  using T = std::ranges::range_value_t<V>;
  T n = 0;
  for (int i = 0; i < N; ++i) {
    n += a[i] * a[i];
  }
  return n;
}

template <AbstractFixedSizeVector V>
  requires requires(V v) { v.squaredNorm(); }
auto normSquared(const V& a) {
  return a.squaredNorm();
}

auto norm(const AbstractFixedSizeVector auto& a) {
  return std::sqrt(normSquared(a));
}

// Distance
template <Vec3or4 V>
auto distance(const V& a, const V& b) {
  return norm(a - b);
}

// Normalization
auto normalize(const Vec3or4 auto& a) { return a / norm(a); }

// Dot product
template <AbstractVector3 V>
auto dot(const V& a, const V& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <AbstractVector4 V>
auto dot(const V& a, const V& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// Angle between two vectors
template <Vec3or4 V>
auto angleBetween(const V& a, const V& b) {
  using T = std::ranges::range_value_t<V>;
  V a_normalized = normalize(a);
  V b_normalized = normalize(b);

  // If the difference between the vectors is small, acos amplifies numerical
  // rounding errors from the dot product.
  // We can use a small angle approximation: radius * angle = chord length
  // where the radius is forced to 1 by normalizing the vectors, and the chord
  // length is the length of the difference between the unit vectors.
  T diff = norm(a_normalized - b_normalized);
  // TODO: use `SmallAngleTolerance<T>()`
  if (diff < 100 * std::numeric_limits<T>::epsilon()) {
    return diff;
  }
  return std::acos(dot(a_normalized, b_normalized));
}

// Cross product
template <Vec3 V>
V cross(const V& a, const V& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

}  // namespace se3
