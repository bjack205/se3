#pragma once

#include <concepts>

#include "vector_concepts.hpp"

namespace se3 {

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat3 = requires(M m, M other) {
  { m(0, 0) } -> std::convertible_to<T>;
  { m(0, 1) } -> std::convertible_to<T>;
  { m(0, 2) } -> std::convertible_to<T>;
  { m(1, 0) } -> std::convertible_to<T>;
  { m(1, 1) } -> std::convertible_to<T>;
  { m(1, 2) } -> std::convertible_to<T>;
  { m(2, 0) } -> std::convertible_to<T>;
  { m(2, 1) } -> std::convertible_to<T>;
  { m(2, 2) } -> std::convertible_to<T>;
  { m == other } -> std::convertible_to<bool>;
  { m != other } -> std::convertible_to<bool>;
  { M::identity() } -> std::convertible_to<M>;
};

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat4 = requires(M m, M other) {
  { m(0, 0) } -> std::convertible_to<T>;
  { m(0, 1) } -> std::convertible_to<T>;
  { m(0, 2) } -> std::convertible_to<T>;
  { m(0, 3) } -> std::convertible_to<T>;
  { m(1, 0) } -> std::convertible_to<T>;
  { m(1, 1) } -> std::convertible_to<T>;
  { m(1, 2) } -> std::convertible_to<T>;
  { m(1, 3) } -> std::convertible_to<T>;
  { m(2, 0) } -> std::convertible_to<T>;
  { m(2, 1) } -> std::convertible_to<T>;
  { m(2, 2) } -> std::convertible_to<T>;
  { m(2, 3) } -> std::convertible_to<T>;
  { m(3, 0) } -> std::convertible_to<T>;
  { m(3, 1) } -> std::convertible_to<T>;
  { m(3, 2) } -> std::convertible_to<T>;
  { m(3, 3) } -> std::convertible_to<T>;
  { m == other } -> std::convertible_to<bool>;
  { m != other } -> std::convertible_to<bool>;
  { M::identity() } -> std::convertible_to<M>;
};

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Diag3 = std::ranges::random_access_range<M> and SizeAtCompileTime<M>() == 3 and requires(M m, M other) {
  { m[0] } -> std::convertible_to<T>;
  { m[1] } -> std::convertible_to<T>;
  { m[2] } -> std::convertible_to<T>;
  { m == other } -> std::convertible_to<bool>;
  { m != other } -> std::convertible_to<bool>;
  { M::identity() } -> std::convertible_to<M>;
};

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Diag4 = std::ranges::random_access_range<M> and SizeAtCompileTime<M>() == 4 and requires(M m, M other) {
  { m[0] } -> std::convertible_to<T>;
  { m[1] } -> std::convertible_to<T>;
  { m[2] } -> std::convertible_to<T>;
  { m[3] } -> std::convertible_to<T>;
  { m == other } -> std::convertible_to<bool>;
  { m != other } -> std::convertible_to<bool>;
  { M::identity() } -> std::convertible_to<M>;
};

// TODO: Uniform scaling



}  // namespace se3