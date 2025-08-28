//
// Created by Brian Jackson on 8/27/25.
// Copyright (c) 2025. All rights reserved.
//

#include <algorithm>

#include "gtest/gtest.h"
#include "se3/generic.hpp"
#include "se3/linear_algebra/vector_ops.hpp"

TEST(Examples, LinearAlgebra) {
  se3::Vector3d v{1, 2, 3};

  // Direct access (most efficient)
  EXPECT_EQ(v.x, 1);
  EXPECT_EQ(v.y, 2);
  EXPECT_EQ(v.z, 3);

  // Indexing
  EXPECT_EQ(v[0], 1);
  EXPECT_EQ(v[1], 2);
  EXPECT_EQ(v[2], 3);

  // Range compatibility.
  for (auto& e : v) {
    e *= 2;
  }

  // Initializers
  auto v_x = se3::UnitX<se3::Vector3d>();
  auto v_y = se3::UnitY<se3::Vector3d>();
  auto v_z = se3::UnitZ<se3::Vector3d>();
  auto v_zero = se3::Zero<se3::Vector3d>();
  auto v_ones = se3::Ones<se3::Vector3d>();
  auto v_const = se3::Constant<se3::Vector3d>(2.0);
  auto v_seq = se3::Sequence<se3::Vector3d>(0.0);
  EXPECT_EQ(v_seq, se3::Vector3d(0.0, 1.0, 2.0));
  auto v_seq2 = se3::Sequence<se3::Vector3d>(1.0, 2.0);
  EXPECT_EQ(v_seq2, se3::Vector3d(1.0, 3.0, 5.0));

  // Convenient specializations (provided by "se3/generic.hpp")
  v_x = se3::UnitX<double>();  // equivalently, se3::UnitX();
  v_y = se3::UnitY<double>();
  v_z = se3::UnitZ<double>();

  // Random
  std::mt19937 gen(0);
  auto v_uniform = se3::rand<se3::Vector3d>(gen);
  auto v_normal = se3::randn<se3::Vector3d>(gen);

  // Setters
  se3::setValues(v, {2, 3, 4});
  se3::setUnitX(v);
  se3::setUnitY(v);
  se3::setUnitZ(v);
  se3::setZero(v);
  se3::setOnes(v);
  se3::setConstant(v, 2.0);
  se3::setSequence(v, 0.0);
  se3::setSequence(v, 1.0, 2.0);

  // Scalar arithmetic
  // NOTE: Addition and subtraction with scalars is not supported.
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  auto x = se3::rand<se3::Vector3d>(gen);
  double c = dist(gen);
  auto y = x * c;
  y = x / c;
  y = c * x;
  x *= c;
  x /= c;

  // Vector arithmetic
  // NOTE: Multiplication and division between vectors is not supported.
  auto z = se3::Zero<se3::Vector3d>();
  z = x + y;
  z = x - y;
  x += y;
  x -= y;

  // Norms
  double n_2 = se3::norm(x);
  double n_2sq = se3::normSquared(x);
  double n_inf = se3::normInf(x);
  double n_1 = se3::norm1(x);
  auto x_normalized = se3::normalize(x);
  c = se3::distance(x, y);

  // Products
  z = se3::cross(x, y);
  c = se3::dot(x, y);

  // Other
  c = se3::angleBetween(x, y);
}

TEST(Examples, Quaternions) {
  // Initializers
  se3::Quaterniond q{1, 2, 3, 4};
  auto qI = se3::identity<se3::Quaterniond>();
  se3::Quaterniond q_expm = se3::expm(se3::UnitX() * 0.1);
  auto q_x = se3::RotX<double>(0.1);
  auto q_y = se3::RotX<double>(0.1);
  auto q_z = se3::RotZ<double>(0.1);

  // Composition
  q = q_x * q_y;
  q = se3::compose(q_x, q_y);
  q_y = q * se3::inverse(q_x);

  // Vector rotation
  se3::Vector3d v{1, 2, 3};
  auto v_rot = q * v;
  v = se3::inverse(q) * v;
}
