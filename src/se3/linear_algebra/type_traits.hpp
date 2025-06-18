#pragma once

#include "matrices.hpp"
#include "vectors.hpp"
#include "generic/matrices_generic.hpp"

namespace se3 {

// Primary template for mapping Vec3 types to Mat3 types
template <Vec3 V>
struct Mat3TypeFor {
  // By default, use Matrix3 with the same scalar type as the vector
  using type = generic::Matrix3<std::ranges::range_value_t<V>>;
};

// Helper alias template
template <Vec3 V>
using Mat3TypeFor_t = typename Mat3TypeFor<V>::type;

} // namespace se3
