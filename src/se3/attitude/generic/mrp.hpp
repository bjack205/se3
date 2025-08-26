//
// Created by Brian Jackson on 7/17/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include <concepts>

#include "se3/linear_algebra/generic/vectors_generic.hpp"

namespace se3::generic {

template <std::floating_point T>
struct MRP : Vector3<T> {
  static constexpr bool IsMRP = true;
};

}  // namespace se3::generic