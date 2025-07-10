#pragma once

#include "vector_concepts.hpp"
#include "vectors.hpp"

namespace se3 {

// TODO: Make Unit vectors special types

// Initialization
template <Vec3 V>
V UnitX() {
  return {1.0, 0.0, 0.0};
}

template <Vec3 V>
V UnitY() {
  return {0.0, 1.0, 0.0};
}

template <Vec3 V>
V UnitZ() {
  return {0.0, 0.0, 1.0};
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

// Compound assignment operators for Vec3
template <Vec3 V>
V& operator+=(V& x, const V& y) {
  x.x += y.x;
  x.y += y.y;
  x.z += y.z;
  return x;
}

template <Vec3 V>
V& operator-=(V& x, const V& y) {
  x.x -= y.x;
  x.y -= y.y;
  x.z -= y.z;
  return x;
}

template <Vec3 V>
V& operator*=(V& x, const std::ranges::range_value_t<V>& c) {
  x.x *= c;
  x.y *= c;
  x.z *= c;
  return x;
}

template <Vec3 V>
V& operator/=(V& x, const std::ranges::range_value_t<V>& c) {
  x[0] /= c;
  x[1] /= c;
  x[2] /= c;
  return x;
}

template <Vec4 V, std::floating_point T = std::ranges::range_value_t<V>>
void setValues(V& v, const std::tuple<T, T, T, T>& values) {
  v.x = std::get<0>(values);
  v.y = std::get<1>(values);
  v.z = std::get<2>(values);
  v.w = std::get<2>(values);
}

template <Vec4 V, std::floating_point T>
void setConstant(V& v, T val) {
  setValues(v, {val, val, val, val});
}

// Compound assignment operators for Vec4
template <Vec4 V>
V& operator+=(V& x, const V& y) {
  x[0] += y[0];
  x[1] += y[1];
  x[2] += y[2];
  x[3] += y[3];
  return x;
}

template <Vec4 V>
V& operator-=(V& x, const V& y) {
  x[0] -= y[0];
  x[1] -= y[1];
  x[2] -= y[2];
  x[3] -= y[3];
  return x;
}

template <Vec4 V>
V& operator*=(V& x, const std::ranges::range_value_t<V>& c) {
  x[0] *= c;
  x[1] *= c;
  x[2] *= c;
  x[3] *= c;
  return x;
}

template <Vec4 V>
V& operator/=(V& x, const std::ranges::range_value_t<V>& c) {
  x[0] /= c;
  x[1] /= c;
  x[2] /= c;
  x[3] /= c;
  return x;
}

// Vector addition
template <Vec3 V>
V operator+(const V& x, const V& y) {
  return {x[0] + y[0], x[1] + y[1], x[2] + y[2]};
}

template <Vec4 V>
V operator+(const V& x, const V& y) {
  return {x[0] + y[0], x[1] + y[1], x[2] + y[2], x[3] + y[3]};
}

// Vector subtraction
template <Vec3 V>
V operator-(const V& x, const V& y) {
  return {x[0] - y[0], x[1] - y[1], x[2] - y[2]};
}

template <Vec4 V>
V operator-(const V& x, const V& y) {
  return {x[0] - y[0], x[1] - y[1], x[2] - y[2], x[3] - y[3]};
}

// Scalar multiplication (right)
template <Vec3 V>
V operator*(const V& x, const std::ranges::range_value_t<V>& c) {
  return {x[0] * c, x[1] * c, x[2] * c};
}

template <Vec4 V>
V operator*(const V& x, const std::ranges::range_value_t<V>& c) {
  return {x[0] * c, x[1] * c, x[2] * c, x[3] * c};
}

// Scalar multiplication (left)
template <Vec3 V>
V operator*(const std::ranges::range_value_t<V>& c, const V& x) {
  return {c * x[0], c * x[1], c * x[2]};
}

template <Vec4 V>
V operator*(const std::ranges::range_value_t<V>& c, const V& x) {
  return {c * x[0], c * x[1], c * x[2], c * x[3]};
}

// Negation
template <Vec3 V>
V operator-(const V& x) {
  return {-x[0], -x[1], -x[2]};
}

template <Vec4 V>
V operator-(const V& x) {
  return {-x[0], -x[1], -x[2], -x[3]};
}

// Scalar division
template <Vec3 V>
V operator/(const V& x, const std::ranges::range_value_t<V>& c) {
  return {x[0] / c, x[1] / c, x[2] / c};
}

template <Vec4 V>
V operator/(const V& x, const std::ranges::range_value_t<V>& c) {
  return {x[0] / c, x[1] / c, x[2] / c, x[3] / c};
}

// Sum
template <AbstractFixedSizeVector V,
          std::floating_point T = std::ranges::range_value_t<V>>
T sum(const V& x) {
  constexpr auto N = SizeAtCompileTime<V>();
  T n = 0;
  for (int i = 0; i < N; ++i) {
    n += x[i];
  }
  return n;
}

// Norm
template <AbstractFixedSizeVector V>
  requires requires(V v) { v.squaredNorm(); }
auto normSquared(const V& x) {
  return x.squaredNorm();
}

auto norm(const AbstractFixedSizeVector auto& x) {
  return std::sqrt(normSquared(x));
}

// Distance
template <Vec3or4 V>
auto distance(const V& x, const V& y) {
  return norm(x - y);
}

// Normalization
auto normalize(const Vec3or4 auto& x) { return x / norm(x); }

// Dot product
template <AbstractVector3 V>
auto dot(const V& x, const V& y) {
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

template <AbstractVector4 V>
auto dot(const V& x, const V& y) {
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3];
}

// Angle between two vectors
template <Vec3or4 V>
auto angleBetween(const V& x, const V& y) {
  using T = std::ranges::range_value_t<V>;
  V x_normalized = normalize(x);
  V y_normalized = normalize(y);

  // If the difference between the vectors is small, acos amplifies numerical
  // rounding errors from the dot product.
  // We can use a small angle approximation: radius * angle = chord length
  // where the radius is forced to 1 by normalizing the vectors, and the chord
  // length is the length of the difference between the unit vectors.
  T diff = norm(x_normalized - y_normalized);
  if (diff < 100 * std::numeric_limits<T>::epsilon()) {
    return diff;
  }
  return std::acos(dot(x_normalized, y_normalized));
}

// Cross product
template <Vec3 V>
V cross(const V& x, const V& y) {
  return {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
          x[0] * y[1] - x[1] * y[0]};
}

}  // namespace se3
