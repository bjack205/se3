#include <gtest/gtest.h>

#include <algorithm>

#include "se3/linear_algebra/generic/matrices_generic.hpp"
#include "se3/linear_algebra/matmul.hpp"
#include "se3/linear_algebra/matrix_ops.hpp"

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

template <Mat3 M>
void TestMatrixArithmetic() {
  using T = std::ranges::range_value_t<M>;
  using I = UniformScaling<T>;

  // Add uniform scaling
  M A(1, 2, 3, 4, 5, 6, 7, 8, 9);
  M AI = A + I();
  M AI_expected(2, 2, 3, 4, 6, 6, 7, 8, 10);
  EXPECT_EQ(AI, AI_expected);

  // Add uniform scaling with value
  M A2(1, 2, 3, 4, 5, 6, 7, 8, 9);
  T value = 10.0;
  M AI2 = A2 + I(value);
  M AI2_expected(11, 2, 3, 4, 15, 6, 7, 8, 19);
  EXPECT_EQ(AI2, AI2_expected);

  // Add two matrices
  M B(9, 8, 7, 6, 5, 4, 3, 2, 1);
  M C = A + B;
  M C_expected(10, 10, 10, 10, 10, 10, 10, 10, 10);
  EXPECT_EQ(C, C_expected);
}

TEST(MatrixTests, Matrix3_Arithmetic) {
  TestMatrixArithmetic<Matrix3<float>>();
  TestMatrixArithmetic<Matrix3<double>>();
}

template <Mat3 M, Vec3 V>
void TestMatrixMultiplication() {
  using T = std::ranges::range_value_t<M>;
  using I = UniformScaling<T>;
  const T eps = std::numeric_limits<T>::epsilon();
  M A(1, 2, 3, 4, 5, 6, 7, 8, 9);
  V x(1, 2, 3);
  M At(1, 4, 7, 2, 5, 8, 3, 6, 9);

  auto Ax = A * x;
  EXPECT_EQ(Ax.x, 14);
  EXPECT_EQ(Ax.y, 32);
  EXPECT_EQ(Ax.z, 50);

  auto Atx = At * x;
  EXPECT_EQ(Atx.x, 30);
  EXPECT_EQ(Atx.y, 36);
  EXPECT_EQ(Atx.z, 42);

  EXPECT_EQ(At * x, Transpose(A) * x);
  EXPECT_EQ(At * x, transpose(A) * x);

  A = A + I(10.0);
  T detA = determinant(A);
  EXPECT_NEAR(detA, 2320.0, eps);

  // A * inv(A)
  auto AAinv = A * inverse(A);
  EXPECT_EQ(A * inverse(A), M(I()));
}

TEST(MatrixTests, DISABLED_Matrix3_Multiplication) {
  TestMatrixMultiplication<Matrix3<float>, Vector3<float>>();
  TestMatrixMultiplication<Matrix3<double>, Vector3<double>>();
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

TEST(MatrixTests, Mat34) {
  using T = double;
  Matrix34<T> m0;
  EXPECT_EQ(m0.rows(), 3);
  EXPECT_EQ(m0.cols(), 4);
  EXPECT_TRUE(std::ranges::all_of(m0, [](T x) { return x == T(0); }));

  // Initialize from a view
  Matrix34<T> m1(std::views::iota(0, 100) | std::views::take(12) |
                 std::views::transform([](int i) { return T(2 * i); }));
  for (int i = 0; i < m1.size(); ++i) {
    EXPECT_EQ(m1[i], 2 * i);
  }
  for (int c = 0; c < m1.cols(); ++c) {
    for (int r = 0; r < m1.rows(); ++r) {
      EXPECT_EQ(m1(r, c), 2 * (4 * r + c));
    }
  }
}

}  // namespace generic
}  // namespace se3
