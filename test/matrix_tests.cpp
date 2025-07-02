#include <gtest/gtest.h>

#include "se3/linear_algebra/generic/matrices_generic.hpp"
#include "se3/linear_algebra/matmul.hpp"

namespace se3 {
namespace generic {

TEST(MatrixTest, Concepts) {
  EXPECT_TRUE((std::same_as<Mat3TypeFor<Vector3<double>>, Matrix3<double>>));
  EXPECT_TRUE(Mat3<Matrix3<double>>);
  EXPECT_TRUE(Vec3<Vector3<double>>);
}

TEST(MatrixTest, Mat3_Indexing) {
  Matrix3<double> m = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  EXPECT_EQ(m[0], 1);
  EXPECT_EQ(m[1], 2);
  EXPECT_EQ(m[2], 3);
  EXPECT_EQ(m[3], 4);
  EXPECT_EQ(m[4], 5);
  EXPECT_EQ(m[5], 6);
  EXPECT_EQ(m[6], 7);
  EXPECT_EQ(m[7], 8);
  EXPECT_EQ(m[8], 9);
  EXPECT_EQ(m(0, 0), 1);
  EXPECT_EQ(m(0, 1), 2);
  EXPECT_EQ(m(0, 2), 3);
  EXPECT_EQ(m(1, 0), 4);
  EXPECT_EQ(m(1, 1), 5);
  EXPECT_EQ(m(1, 2), 6);
  EXPECT_EQ(m(2, 0), 7);
  EXPECT_EQ(m(2, 1), 8);
  EXPECT_EQ(m(2, 2), 9);
  int i = 1;
  for (auto x : m) {
    EXPECT_EQ(x, i++);
  }
}

TEST(MatrixTest, Mat3_Vec3) {
  Matrix3<double> m = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  Vector3<double> v = {1, 2, 3};
  auto res = m * v;
  EXPECT_EQ(res.x, 14);
  EXPECT_EQ(res.y, 32);
  EXPECT_EQ(res.z, 50);
}

TEST(MatrixTests, Mat4_Concepts) {
  EXPECT_TRUE(Mat4<Matrix4<double>>);
  EXPECT_TRUE((std::same_as<Mat4TypeFor<Vector4<double>>, Matrix4<double>>));
}

TEST(MatrixTests, Mat4_Vec4) {
  Matrix4<double> m = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  Vector4<double> v = {1, 2, 3, 4};
  auto res = m * v;
  EXPECT_EQ(res.x, 30);
  EXPECT_EQ(res.y, 70);
  EXPECT_EQ(res.z, 110);
  EXPECT_EQ(res.w, 150);
}

TEST(MatrixTests, Mat4Transpose_Vec4) {
  Matrix4<double> m0 = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  Matrix4<double> m1 = {1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15, 4, 8, 12, 16};
  Vector4<double> v = {1, 2, 3, 4};
  EXPECT_FALSE(Mat3<Matrix4<double>>);
  // auto mt = Transpose<Matrix4<double>>(m);
  auto mt = Transpose(m0);
  EXPECT_FALSE(mt.ownsData());
  auto mt_2 = Transpose(m0 * m1);
  EXPECT_TRUE(mt_2.ownsData());
  // EXPECT_EQ(mt(0, 1), 5);
  // EXPECT_EQ(mt(1, 0), 2);
  // auto res = Transpose(m) * v;
  // EXPECT_EQ(res.x, 90);
  // EXPECT_EQ(res.y, 100);
  // EXPECT_EQ(res.z, 110);
  // EXPECT_EQ(res.w, 120);
}

}  // namespace generic
}  // namespace se3
