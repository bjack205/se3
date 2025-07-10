//
// Created by Brian Jackson on 7/9/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once
#include <ranges>

#include "se3/linear_algebra/matrices.hpp"

namespace se3 {

template <typename R, typename T = std::ranges::range_value_t<R>>
concept AbstractRotationMatrix = std::ranges::range<R> and Mat3<R> and
                                 std::is_trivial_v<typename R::MatrixGroup>;


}
