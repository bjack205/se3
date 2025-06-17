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

template <typename V> constexpr std::size_t SizeAtCompileTime() {
  return std::tuple_size_v<V>;
}

template <typename V>
  requires requires() { V::extent; }
constexpr std::size_t SizeAtCompileTime() {
  return V::extent;
}

template <typename V>
  requires requires() { V::SizeAtCompileTime; }
constexpr std::size_t SizeAtCompileTime() {
  return V::SizeAtCompileTime;
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

template <AbstractFixedSizeVector V> auto normSquared(const V &x) {
  constexpr auto N = SizeAtCompileTime<V>();
  using T = std::ranges::range_value_t<V>;
  T n = 0;
  for (int i = 0; i < N; ++i) {
    n += x[i] * x[i];
  }
  return n;
}

} // namespace se3