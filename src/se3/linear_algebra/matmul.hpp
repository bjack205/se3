#pragma once

#include "matrices.hpp"
#include "type_traits.hpp"

namespace se3 {

/////////////////////////////////////////////
/// 3x3 Operations
/////////////////////////////////////////////


template <Mat3 M, Vec3 V>
V operator*(const M& m, const V& v) {
  return {m(0, 0) * v[0] + m(0, 1) * v[1] + m(0, 2) * v[2],
          m(1, 0) * v[0] + m(1, 1) * v[1] + m(1, 2) * v[2],
          m(2, 0) * v[0] + m(2, 1) * v[1] + m(2, 2) * v[2]};
}

template <Diag3 D, Vec3 V>
V operator*(const D& d, const V& v) {
  return {d[0] * v[0], d[1] * v[1], d[2] * v[2]};
}

// template <Diag3 D>
// Diag3 operator*(const D& d1, const D& d2) {
//   return {d1[0] * d2[0], d1[1] * d2[1], d1[2] * d2[2]};
// }

template <Diag3 D, Mat3 M>
M operator*(const D& d, const M& m) {
  return {d[0] * m(0, 0), d[0] * m(0, 1), d[0] * m(0, 2),
          d[1] * m(1, 0), d[1] * m(1, 1), d[1] * m(1, 2),
          d[2] * m(2, 0), d[2] * m(2, 1), d[2] * m(2, 2)};
}

template <Mat3 M, Diag3 D>
M operator*(const M& m, const D& d) {
  return {m(0, 0) * d[0], m(0, 1) * d[1], m(0, 2) * d[2],
          m(1, 0) * d[0], m(1, 1) * d[1], m(1, 2) * d[2],
          m(2, 0) * d[0], m(2, 1) * d[1], m(2, 2) * d[2]};
}

template <Mat3 M>
M operator*(const M& m1, const M& m2) {
  return {m1(0, 0) * m2(0, 0) + m1(0, 1) * m2(1, 0) + m1(0, 2) * m2(2, 0),
          m1(0, 0) * m2(0, 1) + m1(0, 1) * m2(1, 1) + m1(0, 2) * m2(2, 1),
          m1(0, 0) * m2(0, 2) + m1(0, 1) * m2(1, 2) + m1(0, 2) * m2(2, 2),
          m1(1, 0) * m2(0, 0) + m1(1, 1) * m2(1, 0) + m1(1, 2) * m2(2, 0),
          m1(1, 0) * m2(0, 1) + m1(1, 1) * m2(1, 1) + m1(1, 2) * m2(2, 1),
          m1(1, 0) * m2(0, 2) + m1(1, 1) * m2(1, 2) + m1(1, 2) * m2(2, 2),
          m1(2, 0) * m2(0, 0) + m1(2, 1) * m2(1, 0) + m1(2, 2) * m2(2, 0),
          m1(2, 0) * m2(0, 1) + m1(2, 1) * m2(1, 1) + m1(2, 2) * m2(2, 1),
          m1(2, 0) * m2(0, 2) + m1(2, 1) * m2(1, 2) + m1(2, 2) * m2(2, 2)};
}

// The following methods default to returning a Matrix3. Need to make sure this can
// be specialized for specific vector types.

template <Vec3 V, std::floating_point T = std::ranges::range_value_t<V>>
Mat3TypeFor_t<V> outerProduct(const V& v1, const V& v2) {
  return {v1[0] * v2[0], v1[0] * v2[1], v1[0] * v2[2],
          v1[1] * v2[0], v1[1] * v2[1], v1[1] * v2[2],
          v1[2] * v2[0], v1[2] * v2[1], v1[2] * v2[2]};
}

template <Vec3 V>
Mat3TypeFor_t<V> skew(const V& v) {
  return {0, -v[2], v[1],
          v[2], 0, -v[0],
          -v[1], v[0], 0};
}

/////////////////////////////////////////////
/// 4x4 Operations
/////////////////////////////////////////////

}  // namespace se3