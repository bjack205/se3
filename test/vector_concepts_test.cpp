//
// Created by Brian Jackson on 3/1/25.
// Copyright (c) 2025. All rights reserved.
//

#include <array>
#include <span>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "se3/attitude/vector_concepts.hpp"

using namespace se3;

TEST(VectorConcepts, SizeAtCompileTime) {
  EXPECT_EQ((SizeAtCompileTime<std::tuple<int,int,int>>()), 3);
  EXPECT_EQ((SizeAtCompileTime<std::array<float,3>>()), 3);
  EXPECT_EQ((SizeAtCompileTime<std::array<int,4>>()), 4);
  EXPECT_EQ((SizeAtCompileTime<std::span<int,3>>()), 3);
  EXPECT_EQ((SizeAtCompileTime<std::span<int,4>>()), 4);
  EXPECT_EQ((SizeAtCompileTime<Eigen::Vector3d>()), 3);
  EXPECT_EQ((SizeAtCompileTime<Eigen::Matrix3i>()), 9);
}

TEST(VectorConcepts, Vec3) {
  EXPECT_TRUE((Vec3<std::span<int,3>>));
  EXPECT_TRUE((Vec3<std::array<int,3>>));
  EXPECT_TRUE((Vec3<std::array<float,3>>));
  EXPECT_TRUE(Vec3<Eigen::Vector3d>);
  EXPECT_TRUE(Vec3<Eigen::Vector3i>);
  EXPECT_TRUE(Vec3<Eigen::Array3f>);
  EXPECT_TRUE((Vec3<Eigen::VectorBlock<Eigen::Array3f, 3>>));
  EXPECT_TRUE((Vec3<Eigen::VectorBlock<Eigen::VectorXd, 3>>));
  EXPECT_TRUE((Vec3<Eigen::Block<Eigen::VectorXd, 3, 1>>));
  EXPECT_TRUE(Vec3<Eigen::Ref<Eigen::Vector3i>>);
  EXPECT_TRUE(Vec3<Eigen::Diagonal<Eigen::Matrix3d>>);
}

TEST(VectorConcepts, Vec4) {
  EXPECT_TRUE((Vec4<std::span<int,4>>));
  EXPECT_TRUE((Vec4<std::array<int,4>>));
  EXPECT_TRUE(Vec4<Eigen::Vector4d>);
  EXPECT_TRUE(Vec4<Eigen::Vector4i>);
  EXPECT_TRUE(Vec4<Eigen::Array4f>);
  EXPECT_TRUE((Vec4<Eigen::VectorBlock<Eigen::Array<float, 5, 1>, 4>>));
  EXPECT_TRUE((Vec4<Eigen::VectorBlock<Eigen::VectorXd, 4>>));
  EXPECT_TRUE((Vec4<Eigen::Block<Eigen::VectorXd, 4, 1>>));
  EXPECT_TRUE(Vec4<Eigen::Ref<Eigen::Vector4i>>);
  EXPECT_TRUE(Vec4<Eigen::Diagonal<Eigen::Matrix4d>>);
}

TEST(VectorConcepts, Norm) {
  const Eigen::Vector3d x(3,4,5);
  EXPECT_DOUBLE_EQ(normSquared(x), 50);
  EXPECT_DOUBLE_EQ(norm(x), std::sqrt(50));
}
