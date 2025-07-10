//
// Created by Brian Jackson on 3/1/25.
// Copyright (c) 2025. All rights reserved.
//

#include "se3/linear_algebra/vector_concepts.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <array>
#include <span>

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
  EXPECT_TRUE((AbstractVector3<std::span<int,3>>));
  EXPECT_TRUE((AbstractVector3<std::array<int,3>>));
  EXPECT_TRUE((AbstractVector3<std::array<float,3>>));
  EXPECT_TRUE(AbstractVector3<Eigen::Vector3d>);
  EXPECT_TRUE(AbstractVector3<Eigen::Vector3i>);
  EXPECT_TRUE(AbstractVector3<Eigen::Array3f>);
  EXPECT_TRUE((AbstractVector3<Eigen::VectorBlock<Eigen::Array3f, 3>>));
  EXPECT_TRUE((AbstractVector3<Eigen::VectorBlock<Eigen::VectorXd, 3>>));
  EXPECT_TRUE((AbstractVector3<Eigen::Block<Eigen::VectorXd, 3, 1>>));
  EXPECT_TRUE(AbstractVector3<Eigen::Ref<Eigen::Vector3i>>);
  EXPECT_TRUE(AbstractVector3<Eigen::Diagonal<Eigen::Matrix3d>>);
}

TEST(VectorConcepts, Vec4) {
  EXPECT_TRUE((AbstractVector4<std::span<int,4>>));
  EXPECT_TRUE((AbstractVector4<std::array<int,4>>));
  EXPECT_TRUE(AbstractVector4<Eigen::Vector4d>);
  EXPECT_TRUE(AbstractVector4<Eigen::Vector4i>);
  EXPECT_TRUE(AbstractVector4<Eigen::Array4f>);
  EXPECT_TRUE((AbstractVector4<Eigen::VectorBlock<Eigen::Array<float, 5, 1>, 4>>));
  EXPECT_TRUE((AbstractVector4<Eigen::VectorBlock<Eigen::VectorXd, 4>>));
  EXPECT_TRUE((AbstractVector4<Eigen::Block<Eigen::VectorXd, 4, 1>>));
  EXPECT_TRUE(AbstractVector4<Eigen::Ref<Eigen::Vector4i>>);
  EXPECT_TRUE(AbstractVector4<Eigen::Diagonal<Eigen::Matrix4d>>);
}
