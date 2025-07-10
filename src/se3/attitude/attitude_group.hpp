//
// Created by Brian Jackson on 7/9/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include "se3/linear_algebra/type_traits.hpp"

namespace se3 {

// Forward declare the generic implementations
namespace generic {
template <std::floating_point T>
struct Quaternion;

template <std::floating_point T>
struct Rot3;

template <std::floating_point T>
struct MRP;

template <std::floating_point T>
struct RodriguesParam;
}  // namespace generic

template <typename MatGroupType, std::floating_point T>
struct AttitudeGroup {
  using Quaternion = generic::Quaternion<T>;
  using Rot3 = generic::Rot3<T>;
  using MRP = generic::MRP<T>;
  using RodriguesParam = generic::RodriguesParam<T>;
};

template <typename A>
using AttitudeGroupFor =
    AttitudeGroup<typename A::MatrixGroup, std::ranges::range_value_t<A>>;

template <typename A>
using QuatTypeFor = typename AttitudeGroupFor<A>::Quaternion;

}  // namespace se3