//
// Created by Brian Jackson on 6/16/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include <concepts>
#include <ranges>

#include "vector_concepts.hpp"

namespace se3 {

template <typename V, typename T = std::ranges::range_value_t<V>>
concept Vec3 = AbstractVector3<V> and requires(V v, V other)
{
  { v[0] } -> std::convertible_to<T>;
  { v[1] } -> std::convertible_to<T>;
  { v[2] } -> std::convertible_to<T>;
  { v.x } -> std::convertible_to<T>;
  { v.y } -> std::convertible_to<T>;
  { v.z } -> std::convertible_to<T>;
  { v == other } -> std::convertible_to<bool>;
  { v != other } -> std::convertible_to<bool>;
};

template <typename V, typename T = std::ranges::range_value_t<V>>
concept Vec4 = AbstractVector4<V> and requires(V v, V other)
{
  { v[0] } -> std::convertible_to<T>;
  { v[1] } -> std::convertible_to<T>;
  { v[2] } -> std::convertible_to<T>;
  { v[3] } -> std::convertible_to<T>;
  { v.x } -> std::convertible_to<T>;
  { v.y } -> std::convertible_to<T>;
  { v.z } -> std::convertible_to<T>;
  { v.w } -> std::convertible_to<T>;
  { v == other } -> std::convertible_to<bool>;
  { v != other } -> std::convertible_to<bool>;
};

template <typename V>
concept Vec3or4 = Vec3<V> or Vec4<V>;



}  // namespace se3