#include <gtest/gtest.h>

#include "se3/attitude/quaternion.hpp"

namespace se3 {
TEST(Quaternion, DefaultConstructor) {
    Quaternion<double> q;
}
}