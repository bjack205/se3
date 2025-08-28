//
// Created by Brian Jackson on 8/27/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include "se3/attitude/generic/quaternion.hpp"

namespace se3 {

using namespace generic;

using Vector3f = Vector3<float>;
using Matrix3f = Matrix3<float>;
using Quaternionf = Quaternion<float>;

using Vector3d = Vector3<double>;
using Matrix3d = Matrix3<double>;
using Quaterniond = Quaternion<double>;

template <std::floating_point T>
Quaternion<T> RotX(T angle) {
  return RotX<Quaternion<T>>(angle);
}

template <std::floating_point T>
Quaternion<T> RotY(T angle) {
  return RotY<Quaternion<T>>(angle);
}

template <std::floating_point T>
Quaternion<T> RotZ(T angle) {
  return RotZ<Quaternion<T>>(angle);
}

template <std::floating_point T = double>
Vector3<T> UnitX() {
  return UnitX<Vector3<T>>();
}

template <std::floating_point T = double>
Vector3<T> UnitY() {
  return UnitY<Vector3<T>>();
}

template <std::floating_point T = double>
Vector3<T> UnitZ() {
  return UnitZ<Vector3<T>>();
}

// Bring operators inside the namespace for ADL to work.
namespace generic {
using se3::operator+;
using se3::operator-;
using se3::operator*;
using se3::operator/;
using se3::operator+=;
using se3::operator-=;
using se3::operator*=;
using se3::operator/=;
}  // namespace generic

}  // namespace se3