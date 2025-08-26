// Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.

#pragma once

#include "se3/linear_algebra/matrices.hpp"

namespace se3 {

template <typename M>
struct Transpose {
  using T = std::ranges::range_value_t<M>;
  using MBase = std::decay_t<M>;

  static constexpr auto RowsAtCompileTime = MBase::ColsAtCompileTime;
  static constexpr auto ColsAtCompileTime = MBase::RowsAtCompileTime;
  static constexpr auto SizeAtCompileTime = MBase::SizeAtCompileTime;

  // Determine the storage type:
  // If T is an l-value reference (e.g., Matrix&), store T directly (a reference).
  // If T is an r-value reference (e.g., Matrix&&), store the decayed type (Matrix by value).
  // If T is by value (e.g., Matrix), store the decayed type (Matrix by value).
  using StorageType = std::conditional_t<std::is_lvalue_reference_v<M>,
                                         M,
                                         std::decay_t<M>>;
  StorageType parent;

  template <typename U>
  Transpose(U&& p) : parent(std::forward<U>(p)) {}

  std::size_t size() const { return parent.size(); }
  T& operator[](std::size_t i) { return parent[i]; }
  const T& operator[](std::size_t i) const { return parent[i]; }

  T* data() { return parent.data(); }
  const T* data() const { return parent.data(); }
  T* begin() { return parent.data(); }
  T* end() { return parent.data() + size(); }
  const T* begin() const { return parent.data(); }
  const T* end() const { return parent.data() + size(); }

  T& operator()(int row, int col) { return parent(col, row); }
  const T& operator()(int row, int col) const { return parent(col, row); }

  bool operator==(const Transpose& rhs) const {
    return parent == rhs.parent;
  }

  bool ownsData() const {
    return not std::is_lvalue_reference_v<M>;
  }

};

template <typename U>
Transpose(U&&) -> Transpose<U>;

// TODO: need a way to assign a `Transpose<M>` back to `M`.
// TODO: need a method for creating a transposed copy.
// TODO: specialize matrix-transpose multiplication.



}  // namespace se3