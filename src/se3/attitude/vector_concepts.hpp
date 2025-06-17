//
// Created by Brian Jackson on 3/1/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include <cstddef>
#include <ranges>
#include <cmath>
#include <tuple>

namespace se3 {

template <typename V>
constexpr std::size_t SizeAtCompileTime() {
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
concept Vec = std::ranges::random_access_range <V> and SizeAtCompileTime <V>() >= 0;

template <typename V, int N, typename T = std::ranges::range_value_t <V> >
concept VecN = std::ranges::random_access_range <V> and SizeAtCompileTime <V>() == N;

template <typename V, typename T = std::ranges::range_value_t <V> >
concept Vec3 = VecN <V, 3>;

template <typename V, typename T = std::ranges::range_value_t <V> >
concept Vec4 = VecN <V, 4>;

template <Vec V>
auto normSquared(const V& x) {
  constexpr auto N = SizeAtCompileTime<V>();
  using T = std::ranges::range_value_t <V>;
  T n = 0;
  for (int i = 0; i < N; ++i) {
    n += x[i] * x[i];
  }
  return n;
}

template <Vec V>
  requires requires(V v) { v.squaredNorm(); }
auto normSquared(const V& x) {
  return x.squaredNorm();
}

auto norm(const Vec auto& x) {
  return std::sqrt(normSquared(x));
}


} // namespace se3