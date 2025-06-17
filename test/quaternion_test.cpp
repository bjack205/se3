#include <gtest/gtest.h>

#include "se3/attitude/quaternion.hpp"

#include <algorithm>

namespace se3 {

TEST(QuaternionTest, VectorInterface) {
  Quaternion<double> q;
  q.w = 0;
  EXPECT_DOUBLE_EQ(q[0], 0.0);
  for (auto el : q) {
    EXPECT_DOUBLE_EQ(el, 0.0);
  }
  std::ranges::all_of(q, [](auto el) { return el == 0.0; });
  EXPECT_TRUE(std::ranges::random_access_range<Quaternion<double>>);
  EXPECT_TRUE(AbstractVector4<Quaternion<double>>);

  EXPECT_TRUE((std::same_as<std::ranges::range_value_t<Quaternion<double>>, double>));
}

}