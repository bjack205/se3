#pragma once

#include "se3/linear_algebra/matrices.hpp"

#include "vectors_generic.hpp"

namespace se3::generic {

template <std::floating_point T>
struct Matrix3 {
  static constexpr int RowsAtCompileTime = 3;
  static constexpr int ColsAtCompileTime = 3;
  static constexpr int SizeAtCompileTime = RowsAtCompileTime * ColsAtCompileTime;

  // TODO: add compile flag to control default constructor (identity or zero)
  Matrix3()
      : m00(1),
        m01(0),
        m02(0),
        m10(0),
        m11(1),
        m12(0),
        m20(0),
        m21(0),
        m22(1) {}

  Matrix3(T m00, T m10, T m20, T m01, T m11, T m21, T m02, T m12, T m22)
      : m00(m00),
        m01(m10),
        m02(m20),
        m10(m01),
        m11(m11),
        m12(m21),
        m20(m02),
        m21(m12),
        m22(m22) {}

  static Matrix3 identity() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }

  static Matrix3 zero() { return {0, 0, 0, 0, 0, 0, 0, 0, 0}; }

  static Matrix3 fromDiagonal(const AbstractVector3<T> auto& v) {
    return {v[0], 0, 0, 0, v[1], 0, 0, 0, v[2]};
  }

  T* data() { return &m00; }
  const T* data() const { return &m00; }

  int rows() const { return RowsAtCompileTime; }
  int cols() const { return ColsAtCompileTime; }

  std::size_t size() const { return RowsAtCompileTime * ColsAtCompileTime; }
  T& operator[](const int i) { return (&m00)[i]; }
  const T& operator[](const int i) const { return (&m00)[i]; }

  T* begin() { return &m00; }
  T* end() { return &m22 + 1; }
  const T* begin() const { return &m00; }
  const T* end() const { return &m22 + 1; }

  T& operator()(const int row, const int col) {
    return (&m00)[col + row * ColsAtCompileTime];
  }
  const T& operator()(const int row, const int col) const {
    return (&m00)[col + row * ColsAtCompileTime];
  }

  auto operator<=>(const Matrix3& other) const = default;

  T m00, m01, m02;
  T m10, m11, m12;
  T m20, m21, m22;
};

template <std::floating_point T>
struct Matrix4 {
  static constexpr int kRowsAtCompileTime = 4;
  static constexpr int kColsAtCompileTime = 4;

  // TODO: add compile flag to control default constructor (identity or zero)
  Matrix4() : m00(1), m10(0), m20(0), m30(0), m01(0), m11(1), m21(0), m31(0), m02(0), m12(0), m22(1), m32(0), m03(0), m13(0), m23(0), m33(1) {}

  static Matrix4 identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

  static Matrix4 zero() { return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; }

  static Matrix4 fromDiagonal(const AbstractVector4<T> auto& v) {
    return {v[0], 0, 0, 0, 0, v[1], 0, 0, 0, 0, v[2], 0, 0, 0, 0, v[3]};
  }

  Vector4<T> diagonal() const { return {m00, m11, m22, m33}; }  

  Vector4<T> row(const int row) const {
    const int offset = row * kColsAtCompileTime;
    return {(&m00)[offset], (&m00)[offset + 1], (&m00)[offset + 2], (&m00)[offset + 3]};
  }

  Vector4<T> col(const int col) const { 
    const T& c0 = (&m00)[col];
    const T& c1 = (&m00)[col + kColsAtCompileTime];
    const T& c2 = (&m00)[col + 2 * kColsAtCompileTime];
    const T& c3 = (&m00)[col + 3 * kColsAtCompileTime];
    return {c0, c1, c2, c3};
  }

  T* data() { return &m00; }
  const T* data() const { return &m00; }

  int rows() const { return kRowsAtCompileTime; }
  int cols() const { return kColsAtCompileTime; }
  std::size_t size() const { return kRowsAtCompileTime * kColsAtCompileTime; }

  T& operator()(const int row, const int col) {
    return (&m00)[col + row * kColsAtCompileTime];
  }
  const T& operator()(const int row, const int col) const {
    return (&m00)[col + row * kColsAtCompileTime];
  }

  auto operator<=>(const Matrix4& other) const = default;

  T m00, m01, m02, m03;
  T m10, m11, m12, m13;
  T m20, m21, m22, m23;
  T m30, m31, m32, m33;
};

template <std::floating_point T>
struct DiagonalMatrix3 {
  static constexpr int kRowsAtCompileTime = 3;
  static constexpr int kColsAtCompileTime = 3;

  DiagonalMatrix3() : m00(1), m11(1), m22(1) {}
  DiagonalMatrix3(T m00, T m11, T m22) : m00(m00), m11(m11), m22(m22) {}
  DiagonalMatrix3(const AbstractVector3<T> auto& v) : m00(v[0]), m11(v[1]), m22(v[2]) {}

  Vector3<T> diagonal() const { return {m00, m11, m22}; }

  T* data() { return &m00; }
  const T* data() const { return &m00; }

  int rows() const { return kRowsAtCompileTime; }
  int cols() const { return kColsAtCompileTime; }
  std::size_t size() const { return kRowsAtCompileTime; }

  T& operator[](const int i) { return (&m00)[i]; }
  const T& operator[](const int i) const { return (&m00)[i]; }

  // NOTE: Diagonal matrix only has diagonal elements. 2D indexing is not supported.
  T& operator()(const int row, const int col) = delete;
  const T& operator()(const int row, const int col) const = delete;

  auto operator<=>(const DiagonalMatrix3& other) const = default;

  T m00, m11, m22;
};


}