//
// Created by Brian Jackson on 6/17/25.
// Copyright (c) 2025. All rights reserved.
//

#include <gtest/gtest.h>

#include <concepts>
#include <ranges>

#include "se3/linear_algebra/simd/vectors_simd.hpp"
#include "se3/linear_algebra/vector_concepts.hpp"
#include "se3/linear_algebra/vector_ops.hpp"
#include "se3/linear_algebra/vectors.hpp"

namespace se3 {

TEST(VectorTests, SizeAtCompileTime) {
  EXPECT_EQ(SizeAtCompileTime<Vector3<double>>(), 3);
  EXPECT_EQ(SizeAtCompileTime<Vector3<float>>(), 3);
  EXPECT_EQ(SizeAtCompileTime<Vector4<double>>(), 4);
  EXPECT_EQ(SizeAtCompileTime<Vector4<float>>(), 4);
}

TEST(VectorTests, Concepts) {
  EXPECT_TRUE(std::ranges::random_access_range<Vector3<double>>);
  EXPECT_TRUE(AbstractFixedSizeVector<Vector3<double>>);
  EXPECT_TRUE(AbstractVector3<Vector3<double>>);
  EXPECT_FALSE(AbstractVector4<Vector3<double>>);

  EXPECT_TRUE(AbstractFixedSizeVector<Vector4<double>>);
  EXPECT_TRUE(AbstractVector4<Vector4<double>>);
  EXPECT_FALSE(AbstractVector3<Vector4<double>>);

  EXPECT_TRUE(Vec3or4<Vector3<double>>);
  EXPECT_TRUE(Vec3or4<Vector4<double>>);

  EXPECT_TRUE(Vec3<Vector3<double>>);
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

  for (auto& el : a) {
    el = 5.0;
  }
  EXPECT_DOUBLE_EQ(sum(a), 15.0);
  std::ranges::fill(a, 3.0);
  EXPECT_DOUBLE_EQ(sum(a), 9.0);

  std::ranges::copy(a, b.begin());
  EXPECT_EQ(a, b);
}

TEST(VectorTests, Vector3_Arithmetic) {
  Vector3 a(1.0, 2.0, 3.0);
  Vector3 b(-2.0, -4.0, 2.0);
  Vector3 c = a + b;
  EXPECT_EQ(c[0], -1.0);
  EXPECT_EQ(c[1], -2.0);
  EXPECT_EQ(c[2], 5.0);

  EXPECT_EQ(c - a, b);
  EXPECT_EQ(c - b, a);
  EXPECT_EQ(a - c, -b);

  EXPECT_EQ(2 * a, a + a);
  EXPECT_EQ(a * 2, a + a);
  EXPECT_EQ(2.0 * a, a + a);
  EXPECT_EQ(a / 0.5, a + a);
}

TEST(VectorTests, Vector3_LinearAlgebra) {
  Vector3 a(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(dot(a, a), normSquared(a));
  EXPECT_DOUBLE_EQ(normSquared(a), 14.0);
  EXPECT_DOUBLE_EQ(norm(a), std::sqrt(14.0));
  EXPECT_DOUBLE_EQ(norm(a), std::sqrt(dot(a, a)));
  EXPECT_DOUBLE_EQ(norm(a), std::sqrt(normSquared(a)));

  EXPECT_EQ(normalize(a), a / norm(a));
  EXPECT_EQ(normalize(a), a / std::sqrt(normSquared(a)));
  EXPECT_EQ(normalize(a) * norm(a), a);

  EXPECT_EQ(angleBetween(a, a), 0.0);
  EXPECT_EQ(angleBetween(a, -a), std::numbers::pi);

  auto e_x = Vector3<double>::UnitX();
  auto e_y = Vector3<double>::UnitY();
  auto e_z = Vector3<double>::UnitZ();
  EXPECT_EQ(cross(e_x, e_y), e_z);
  EXPECT_EQ(cross(e_y, e_z), e_x);
  EXPECT_EQ(dot(e_x, e_y), 0.0);

  EXPECT_EQ(angleBetween(e_x, e_y), std::numbers::pi / 2.0);
  EXPECT_EQ(angleBetween(e_y, e_x), std::numbers::pi / 2.0);
}

TEST(VectorTests, Vector3_SIMD) {
  simd::Vector3<double> a(1.0, 2.0, 3.0);
  EXPECT_EQ(a[0], 1.0);
  EXPECT_EQ(a[1], 2.0);
  EXPECT_EQ(a[2], 3.0);
  EXPECT_EQ(a.x, 1.0);
  EXPECT_EQ(a.y, 2.0);
  EXPECT_EQ(a.z, 3.0);
  EXPECT_EQ(sizeof(a), sizeof(double) * 4);
  EXPECT_TRUE(AbstractVector3<simd::Vector3<double>>);
  EXPECT_TRUE(Vec3<simd::Vector3<double>>);

  auto b = a + a;
  EXPECT_EQ(a * 2.0, a + a);
}

}  // namespace se3