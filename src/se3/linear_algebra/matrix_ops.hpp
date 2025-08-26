// Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.

#pragma once

#include "matrices.hpp"

namespace se3 {

// A / c
template <Mat3 M, std::floating_point T = std::ranges::range_value_t<M>>
M operator/(const M& m, T c) {
  return {m(0, 0) / c, m(0, 1) / c, m(0, 2) / c,
          m(1, 0) / c, m(1, 1) / c, m(1, 2) / c,
          m(2, 0) / c, m(2, 1) / c, m(2, 2) / c};
}

// A + B
template <Mat3 M>
M operator+(const M& m1, const M& m2) {
  return {m1(0, 0) + m2(0, 0), m1(0, 1) + m2(0, 1), m1(0, 2) + m2(0, 2),
          m1(1, 0) + m2(1, 0), m1(1, 1) + m2(1, 1), m1(1, 2) + m2(1, 2),
          m1(2, 0) + m2(2, 0), m1(2, 1) + m2(2, 1), m1(2, 2) + m2(2, 2)};
}

// A + I
template <Mat3 M, std::floating_point T = std::ranges::range_value_t<M>>
M operator+(const M& m, const UniformScaling<T>& I) {
  return {
    m(0, 0) + I.value, m(0, 1), m(0, 2),
    m(1, 0), m(1, 1) + I.value, m(1, 2),
    m(2, 0), m(2, 1), m(2, 2) + I.value
  };
}

// A += B
template <Mat3 M>
M& operator+=(M& m1, const M& m2) {
  m1(0, 0) += m2(0, 0);
  m1(0, 1) += m2(0, 1);
  m1(0, 2) += m2(0, 2);
  m1(1, 0) += m2(1, 0);
  m1(1, 1) += m2(1, 1);
  m1(1, 2) += m2(1, 2);
  m1(2, 0) += m2(2, 0);
  m1(2, 1) += m2(2, 1);
  m1(2, 2) += m2(2, 2);
  return m1;
}

// A += I
template <Mat3 M, std::floating_point T = std::ranges::range_value_t<M>>
M& operator+=(M& m, const UniformScaling<T>& I) {
  m(0, 0) += I.value;
  m(1, 1) += I.value;
  m(2, 2) += I.value;
  return m;
}

// A'
template <Mat3 M>
M transpose(const M& m) {
  // clang-format off
  return {m(0, 0), m(1, 0), m(2, 0),
          m(0, 1), m(1, 1), m(2, 1),
          m(0, 2), m(1, 2), m(2, 2)};
  // clang-format on
}

// det(A)
template <Mat3 M, std::floating_point T = std::ranges::range_value_t<M>>
T determinant(const M& m) {
  return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
         m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
         m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
}

template <Mat3 M>
M adjoint(const M& m) {
  // clang-format off
  return {m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1),
          m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2),
          m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1),
          m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2),
          m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0),
          m(0, 2) * m(1, 0) - m(0, 0) * m(1, 2),
          m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0),
          m(0, 1) * m(2, 0) - m(0, 0) * m(2, 1),
          m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0)};
  // clang-format on
}

template <Mat3 M>
M inverse(const M& m) {
  using T = std::ranges::range_value_t<M>;
  T det = determinant(m);
  if (det == T(0)) {
    // TODO: Throw a SingularMatrixException or similar.
    throw std::runtime_error("Matrix is singular and cannot be inverted.");
  }
  return adjoint(m) / det;
}

template <Mat4 M>
M transpose(const M& m);

template <Mat4 M>
M inverse(const M& m);

}  // namespace se3