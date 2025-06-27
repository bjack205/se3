#include <gtest/gtest.h>

#include "se3/linear_algebra/generic/matrices_generic.hpp"
#include "se3/linear_algebra/matmul.hpp"

namespace se3 {
namespace generic {

TEST(MatrixTest, Concepts) {
  EXPECT_TRUE((std::same_as<Mat3TypeFor_t<Vector3<double>>, Matrix3<double>>));
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

}  // namespace generic
}  // namespace se3
