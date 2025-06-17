#pragma once

#include <concepts>

#include "vector_concepts.hpp"

namespace se3 {

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat3 = requires(M m, M other) {
  { m(0, 0) } -> std::convertible_to<T>;
  { m(0, 1) } -> std::convertible_to<T>;
  { m(0, 2) } -> std::convertible_to<T>;
  { m(1, 0) } -> std::convertible_to<T>;
  { m(1, 1) } -> std::convertible_to<T>;
  { m(1, 2) } -> std::convertible_to<T>;
  { m(2, 0) } -> std::convertible_to<T>;
  { m(2, 1) } -> std::convertible_to<T>;
  { m(2, 2) } -> std::convertible_to<T>;
  { m == other } -> std::convertible_to<bool>;
  { m != other } -> std::convertible_to<bool>;
  { M::Identity() } -> std::convertible_to<M>;
};

template <std::floating_point T>
struct Matrix3 {
  static constexpr int kRowsAtCompileTime = 3;
  static constexpr int kColsAtCompileTime = 3;

  // TODO: add compile flag to control default constructor (identity or zero)
  Matrix3()
      : m00(1),
        m10(0),
        m20(0),
        m01(0),
        m11(1),
        m21(0),
        m02(0),
        m12(0),
        m22(1) {}

  static Matrix3 Identity() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }

  static Matrix3 Zero() { return {0, 0, 0, 0, 0, 0, 0, 0, 0}; }

  static Matrix3 FromDiagonal(const AbstractVector3<T> auto& v) {
    return {v[0], 0, 0, 0, v[1], 0, 0, 0, v[2]};
  }

  T* data() { return &m00; }
  const T* data() const { return &m00; }

  int rows() const { return kRowsAtCompileTime; }
  int cols() const { return kColsAtCompileTime; }
  std::size_t size() const { return kRowsAtCompileTime * kColsAtCompileTime; }

  T& operator()(const int row,const  int col) {
    return (&m00)[row + col * kRowsAtCompileTime];
  }
  const T& operator()(const int row, const int col) const {
    return (&m00)[row + col * kRowsAtCompileTime];
  }

  auto operator<=>(const Matrix3& other) const = default;

  T m00, m10, m20;
  T m01, m11, m21;
  T m02, m12, m22;
};

}  // namespace se3