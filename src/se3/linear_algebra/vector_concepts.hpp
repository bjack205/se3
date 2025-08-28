//
// Created by Brian Jackson on 3/1/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include <cmath>
#include <cstddef>
#include <ranges>
#include <tuple>

namespace se3 {

template <typename V>
constexpr std::size_t SizeAtCompileTime() {
  using U = std::remove_cvref_t<V>;
  if constexpr (requires { U::SizeAtCompileTime; }) {
    return U::SizeAtCompileTime;
  } else if constexpr (requires { U::extent; }) {
    return U::extent;
  } else {
    return std::tuple_size_v<U>;
  }
}

template <typename V>
concept AbstractFixedSizeVector =
    std::ranges::random_access_range<V> and SizeAtCompileTime<V>() >= 0;

template <typename V, int N, typename T = std::ranges::range_value_t<V>>
concept AbstractFixedSizeVectorN =
    std::ranges::random_access_range<V> and SizeAtCompileTime<V>() == N;

template <typename V, typename T = std::ranges::range_value_t<V>>
concept AbstractVector3 = AbstractFixedSizeVectorN<V, 3>;

template <typename V, typename T = std::ranges::range_value_t<V>>
concept AbstractVector4 = AbstractFixedSizeVectorN<V, 4>;

} // namespace se3