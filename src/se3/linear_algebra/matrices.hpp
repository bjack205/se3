#pragma once

#include <concepts>

#include "vector_concepts.hpp"

namespace se3 {

template <typename M, int R, int C, typename T = std::ranges::range_value_t<M>>
concept FixedSizedMat = std::ranges::random_access_range<M> and
                        M::RowsAtCompileTime == R and M::ColsAtCompileTime == C;

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat3 = FixedSizedMat<M, 3, 3> and requires(M m, M other) {
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
};

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat4 = FixedSizedMat<M, 4, 4> and requires(M m, M other) {
  // TODO: replace indexing with member access
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
};

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Diag3 = FixedSizedMat<M, 3, 3> and SizeAtCompileTime<M>() == 3 and
                requires(M m, M other) {
                  { m[0] } -> std::convertible_to<T>;
                  { m[1] } -> std::convertible_to<T>;
                  { m[2] } -> std::convertible_to<T>;
                  { m == other } -> std::convertible_to<bool>;
                  { m != other } -> std::convertible_to<bool>;
                  { M::identity() } -> std::convertible_to<M>;
                };

template <typename M, typename T = std::ranges::range_value_t<M>>
concept Diag4 = FixedSizedMat<M, 4, 4> and
                SizeAtCompileTime<M>() == 4 and requires(M m, M other) {
                  { m[0] } -> std::convertible_to<T>;
                  { m[1] } -> std::convertible_to<T>;
                  { m[2] } -> std::convertible_to<T>;
                  { m[3] } -> std::convertible_to<T>;
                  { m == other } -> std::convertible_to<bool>;
                  { m != other } -> std::convertible_to<bool>;
                  { M::identity() } -> std::convertible_to<M>;
                };
template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat34 = FixedSizedMat<M, 3, 4>;


template <typename M, typename T = std::ranges::range_value_t<M>>
concept Mat43 = FixedSizedMat<M, 4, 3>;


/**
 * [ xx       ]
 * [ yx yy    ]
 * [ zx zy zz ]
 */
template <typename M, typename T = std::ranges::range_value_t<M>>
concept LTri3 = FixedSizedMat<M, 3, 3> and requires(M m) {
  { m.xx } -> std::convertible_to<T>;
  { m.yx } -> std::convertible_to<T>;
  { m.yy } -> std::convertible_to<T>;
  { m.zx } -> std::convertible_to<T>;
  { m.zy } -> std::convertible_to<T>;
  { m.zz } -> std::convertible_to<T>;
};

/**
 * [ xx xy xz ]
 * [    yy yz ]
 * [       zz ]
 */
template <typename M, typename T = std::ranges::range_value_t<M>>
concept UTri3 = FixedSizedMat<M, 3, 3> and requires(M m) {
  { m.xx } -> std::convertible_to<T>;
  { m.xy } -> std::convertible_to<T>;
  { m.xz } -> std::convertible_to<T>;
  { m.yy } -> std::convertible_to<T>;
  { m.yz } -> std::convertible_to<T>;
  { m.zz } -> std::convertible_to<T>;
};

// TODO: Uniform scaling
template <std::floating_point T>
struct UniformScaling {
  T value = 1;

  template <Mat3 M>
  operator M() const {
    return {value, 0, 0, 0, value, 0, 0, 0, value};
  }
};

// Convert UniformScaling to a dense matrix

}  // namespace se3