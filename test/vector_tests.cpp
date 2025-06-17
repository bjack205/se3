//
// Created by Brian Jackson on 6/17/25.
// Copyright (c) 2025. All rights reserved.
//


#include <gtest/gtest.h>

#include "se3/attitude/vectors.hpp"
#include "se3/attitude/vector_concepts.hpp"

namespace se3 {

TEST(VectorTests, SizeAtCompileTime) {
  EXPECT_EQ(SizeAtCompileTime<Vector3<double>>(), 3);
  EXPECT_EQ(SizeAtCompileTime<Vector3<float>>(), 3);
}

TEST(VectorTests, Vector3) {
  Vector3 a(1.0, 2.0, 3.0);
  Vector3 b(-2.0, -4.0, 2.0);
  EXPECT_EQ(a[0], 1.0);
  EXPECT_EQ(a[1], 2.0);
  EXPECT_EQ(a[2], 3.0);
  a[0] = 2;

  const auto& a_ref = a;
  EXPECT_EQ(a_ref[0], 2.0);
  EXPECT_EQ(a_ref[1], 2.0);
  EXPECT_EQ(a_ref[2], 3.0);

  for (auto &el : a) {
    el = 5.0;
  }
}

}