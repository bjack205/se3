#pragma once

#include "vectors.hpp"
#include "vector_concepts.hpp"

namespace se3 {

// Vector addition
template <AbstractVector3 V>
V operator+(const V &x, const V &y) {
  return {x[0] + y[0], x[1] + y[1], x[2] + y[2]};
}

template <AbstractVector4 V>
V operator+(const V &x, const V &y) {
  return {x[0] + y[0], x[1] + y[1], x[2] + y[2], x[3] + y[3]};
}

// Vector subtraction
template <AbstractVector3 V>
V operator-(const V &x, const V &y) {
  return {x[0] - y[0], x[1] - y[1], x[2] - y[2]};
}

template <AbstractVector4 V>
V operator-(const V &x, const V &y) {
  return {x[0] - y[0], x[1] - y[1], x[2] - y[2], x[3] - y[3]};
}

// Scalar multiplication (right)
template <AbstractVector3 V, typename Scalar>
  requires(std::is_arithmetic_v<Scalar>)
V operator*(const V &x, Scalar c) {
  return {x[0] * c, x[1] * c, x[2] * c};
}

template <AbstractVector4 V, typename Scalar>
  requires(std::is_arithmetic_v<Scalar>)
V operator*(const V &x, Scalar c) {
  return {x[0] * c, x[1] * c, x[2] * c, x[3] * c};
}

// Scalar multiplication (left)
template <AbstractVector3 V, typename Scalar>
  requires(std::is_arithmetic_v<Scalar>)
V operator*(Scalar c, const V &x) {
  return {x[0] * c, x[1] * c, x[2] * c};
}

template <AbstractVector4 V, typename Scalar>
  requires(std::is_arithmetic_v<Scalar>)
V operator*(Scalar c, const V &x) {
  return {x[0] * c, x[1] * c, x[2] * c, x[3] * c};
}

// Negation
template <AbstractVector3 V>
V operator-(const V &x) {
  return {-x[0], -x[1], -x[2]};
}

template <AbstractVector4 V>
V operator-(const V &x) {
  return {-x[0], -x[1], -x[2], -x[3]};
}

// Scalar division
template <AbstractVector3 V, typename Scalar>
  requires(std::is_arithmetic_v<Scalar>)
V operator/(const V &x, Scalar c) {
  return {x[0] / c, x[1] / c, x[2] / c};
}

template <AbstractVector4 V, typename Scalar>
  requires(std::is_arithmetic_v<Scalar>)
V operator/(const V &x, Scalar c) {
  return {x[0] / c, x[1] / c, x[2] / c, x[3] / c};
}

// Sum
template <AbstractFixedSizeVector V, std::floating_point T = std::ranges::range_value_t<V>>
T sum(const V &x) {
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
auto normSquared(const V &x) {
  return x.squaredNorm();
}

auto norm(const AbstractFixedSizeVector auto &x) { return std::sqrt(normSquared(x)); }


// Normalization
auto normalize(const Vec3or4 auto &x) {
  return x / norm(x);
}

// Dot product
template <AbstractVector3 V>
auto dot(const V&x, const V &y) {
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

template <AbstractVector4 V>
auto dot(const V &x, const V &y) {
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3];
}

// Angle between two vectors
template <Vec3or4 V>
auto angleBetween(const V &x, const V &y) {
  return std::acos(dot(normalize(x), normalize(y)));
}

// Cross product
template <std::floating_point T>
Vector3<T> cross(const Vector3<T> &x, const Vector3<T> &y) {
  return {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
          x[0] * y[1] - x[1] * y[0]};
}

}  // namespace se3
