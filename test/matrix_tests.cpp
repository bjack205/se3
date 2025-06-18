#include <gtest/gtest.h>

#include "se3/linear_algebra/generic/matrices_generic.hpp"
#include "se3/linear_algebra/matmul.hpp"

namespace se3 {
namespace generic {

TEST(MatrixTest, MatrixMultiplication) {
  Matrix3<double> m1;
  EXPECT_TRUE((std::same_as<Mat3TypeFor_t<Vector3<double>>, Matrix3<double>>));
}

}  // namespace generic
}  // namespace se3
