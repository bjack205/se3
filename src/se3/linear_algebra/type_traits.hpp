#pragma once

#include "matrices.hpp"
#include "vectors.hpp"

namespace se3 {

struct Generic {};

namespace generic {
template <std::floating_point T>
struct Vector3;

template <std::floating_point T>
struct Vector4;

template <std::floating_point T>
struct Matrix3;

template <std::floating_point T>
struct Matrix4;

template <std::floating_point T>
struct DiagonalMatrix3;
}  // namespace generic

template <typename MatGroupType, std::floating_point T>
struct MatGroup {
  using Vec3 = generic::Vector3<T>;
  using Vec4 = generic::Vector4<T>;
  using Mat3 = generic::Matrix3<T>;
  using Mat4 = generic::Matrix4<T>;
  using Diag3 = generic::DiagonalMatrix3<T>;
  // Diag4
  // Mat34
  // Mat43
  // UTri3
  // LTri3
  // UTri4
  // LTri4
};

template <typename V>
using MatrixGroupFor =
    MatGroup<typename V::MatrixGroup, std::ranges::range_value_t<V>>;

template <typename V>
using Mat3TypeFor = typename MatrixGroupFor<V>::Mat3;

template <typename V>
using Mat4TypeFor = typename MatrixGroupFor<V>::Mat4;

template <typename V>
using Vec3TypeFor = typename MatrixGroupFor<V>::Vec3;

template <typename V>
using Vec4TypeFor = typename MatrixGroupFor<V>::Vec4;

}  // namespace se3
