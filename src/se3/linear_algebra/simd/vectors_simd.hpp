#pragma once

#include <immintrin.h>

#include <array>
#include <concepts>
#include <cstddef>

#include "se3/linear_algebra/vector_concepts.hpp"
#include "se3/linear_algebra/generic/vectors_generic.hpp"


namespace se3 {

// SIMD matrix group
struct SIMD;

namespace simd {

template <std::floating_point T>
struct Vector3 {};

template <>
struct alignas(32) Vector3<double> {
  using Scalar = double;
  using MatrixGroup = SIMD;

  static constexpr std::size_t SizeAtCompileTime = 3;

  union {
    __m256d data_;
    struct {
      double x, y, z;
    };
    double arr[3];
  };

  // Constructors
  Vector3() : data_(_mm256_setzero_pd()) {}
  Vector3(double x_, double y_, double z_)
      : data_(_mm256_set_pd(0.0, z_, y_, x_)) {}

  explicit Vector3(const AbstractVector3 auto& arr_)
      : data_(_mm256_set_pd(0.0, arr_[2], arr_[1], arr_[0])) {}

  explicit Vector3(__m256d data_) : data_(data_) {}

  // operator[]
  double& operator[](int i) { return arr[i]; }
  const double& operator[](int i) const { return arr[i]; }

  // data(), begin(), end()
  double* data() { return arr; }
  const double* data() const { return arr; }
  double* begin() { return arr; }
  double* end() { return arr + 3; }
  const double* begin() const { return arr; }
  const double* end() const { return arr + 3; }

  // Comparison
  bool operator==(const Vector3& other) const {
    __m256d cmp = _mm256_cmp_pd(data_, other.data_, _CMP_EQ_OQ);
    int mask = _mm256_movemask_pd(cmp);
    return (mask & 0x7) == 0x7;  // Only check first 3 elements
  }
  bool operator!=(const Vector3& other) const { return !(*this == other); }
};

template <std::floating_point T>
auto operator+(const Vector3<T> &x, const Vector3<T> &y) {
  __m256d sum = _mm256_add_pd(x.data_, y.data_);
  return Vector3<T>(sum);
}

}  // namespace simd
}  // namespace se3