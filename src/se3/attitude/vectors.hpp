//
// Created by Brian Jackson on 6/16/25.
// Copyright (c) 2025. All rights reserved.
//

#pragma once

#include <concepts>
#include <vector>

namespace se3 {

namespace internal {


}

int foo() {
  std::vector<int> v;
}

template <std::floating_point T>
struct Vector3 {
  static constexpr int SizeAtCompileTime = 3;
  using Scalar = T;

  T& operator[](int i) { return (&x)[i]; }
  const T& operator[](int i) const { return (&x)[i]; }

  T* data() { return &x; }
  const T* data() const { return &x; }

  T* begin() { return &x; }
  T* end() { return &x + SizeAtCompileTime; }
  const T* begin() const { return &x; }
  const T* end() const { return &x; }

  T x, y, z;
};

template <std::floating_point T>
struct Vector4 {
  T x, y, z, w;
};

}