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
  { V::UnitX() } -> std::convertible_to<V>;
  { V::UnitY() } -> std::convertible_to<V>;
  { V::UnitZ() } -> std::convertible_to<V>;
  { V::Zero() } -> std::convertible_to<V>;
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


template <std::floating_point T>
struct Vector3 {
  static constexpr std::size_t SizeAtCompileTime = 3;
  using Scalar = T;

  static Vector3 Zero() { return {0, 0, 0}; }
  static Vector3 UnitX() { return {1, 0, 0}; }
  static Vector3 UnitY() { return {0, 1, 0}; }
  static Vector3 UnitZ() { return {0, 0, 1}; }

  T& operator[](int i) { return (&x)[i]; }
  const T& operator[](int i) const { return (&x)[i]; }

  T* data() { return &x; }
  const T* data() const { return &x; }

  T* begin() { return &x; }
  T* end() { return &x + SizeAtCompileTime; }
  const T* begin() const { return &x; }
  const T* end() const { return &x + SizeAtCompileTime; }

  auto operator<=>(const Vector3& other) const = default;

  T x, y, z;
};

template <std::floating_point T>
struct Vector4 {
  static constexpr std::size_t SizeAtCompileTime = 4;
  using Scalar = T;

  T& operator[](int i) { return (&x)[i]; }
  const T& operator[](int i) const { return (&x)[i]; }

  T* data() { return &x; }
  const T* data() const { return &x; }

  T* begin() { return &x; }
  T* end() { return &x + SizeAtCompileTime; }
  const T* begin() const { return &x; }
  const T* end() const { return &x + SizeAtCompileTime; }

  auto operator<=>(const Vector4& other) const = default;

  T x, y, z, w;
};

}  // namespace se3