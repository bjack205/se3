//
// Created by Brian Jackson on 6/17/25.
// Copyright (c) 2025. All rights reserved.
//

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <concepts>
#include <ranges>
#include <span>

#include "se3/linear_algebra/generic/vectors_generic.hpp"
#include "se3/linear_algebra/simd/matgroup_simd.hpp"
#include "se3/linear_algebra/type_traits.hpp"
#include "se3/linear_algebra/vector_concepts.hpp"
#include "se3/linear_algebra/vector_ops.hpp"
#include "se3/linear_algebra/vectors.hpp"

namespace se3 {

template <Vec3 V>
void testVector3_Concepts() {
  EXPECT_EQ(SizeAtCompileTime<V>(), 3);
  EXPECT_TRUE(std::ranges::random_access_range<V>);
  EXPECT_TRUE(AbstractFixedSizeVector<V>);
  EXPECT_TRUE(AbstractVector3<V>);
  EXPECT_FALSE(AbstractVector4<V>);
  EXPECT_TRUE(Vec3<V>);
  EXPECT_TRUE(Vec3or4<V>);
  EXPECT_TRUE(std::is_trivially_copyable_v<V>);
  EXPECT_TRUE(std::regular<V>);
}

template <Vec4 V>
void testVector4_Concepts() {
  EXPECT_EQ(SizeAtCompileTime<V>(), 4);
  EXPECT_TRUE(std::ranges::random_access_range<V>);
  EXPECT_TRUE(AbstractFixedSizeVector<V>);
  EXPECT_FALSE(AbstractVector3<V>);
  EXPECT_TRUE(Vec4<V>);
  EXPECT_TRUE(Vec3or4<V>);
  EXPECT_TRUE(std::is_trivially_copyable_v<V>);
  EXPECT_TRUE(std::regular<V>);
}

template <Vec3 V, std::floating_point T = std::ranges::range_value_t<V>>
void testVector3_Constructor() {
  // Default
  V v0;
  EXPECT_EQ(v0[0], 0.0);
  EXPECT_EQ(v0[1], 0.0);
  EXPECT_EQ(v0[2], 0.0);

  // Scalar constructor
  V v1(1.0, 2.0, 3.0);
  EXPECT_EQ(v1[0], 1.0);
  EXPECT_EQ(v1[1], 2.0);
  EXPECT_EQ(v1[2], 3.0);

  // Span constructor
  std::vector<T> vec = {2.0, 3.0, 4.0, 5.0};
  V v2{std::span<T, 3>(vec)};
  EXPECT_EQ(v2[0], 2.0);
  EXPECT_EQ(v2[1], 3.0);
  EXPECT_EQ(v2[2], 4.0);

  // Initializer list assignment
  V v3 = {-1.0, 2.0, -3.0};
  EXPECT_EQ(v3[0], -1.0);
  EXPECT_EQ(v3[1], 2.0);
  EXPECT_EQ(v3[2], -3.0);

  // Initializer list
  V v4 = {-1.0, 2.0, -3.0};
  EXPECT_EQ(v4[0], -1.0);
  EXPECT_EQ(v4[1], 2.0);
  EXPECT_EQ(v4[2], -3.0);

  // Subspan
  V v5 = std::span(vec).template subspan<1, 3>();
  EXPECT_EQ(v5[0], 3.0);
  EXPECT_EQ(v5[1], 4.0);
  EXPECT_EQ(v5[2], 5.0);

  // First subspan
  V v6 = std::span(vec).template first<3>();
  EXPECT_EQ(v6[0], 2.0);
  EXPECT_EQ(v6[1], 3.0);
  EXPECT_EQ(v6[2], 4.0);

  // From Eigen
  Eigen::Vector3<T> eigen_v = {1.0, 2.0, 3.0};
  V v7 = eigen_v;
  EXPECT_EQ(v7[0], 1.0);
  EXPECT_EQ(v7[1], 2.0);
  EXPECT_EQ(v7[2], 3.0);
}

template <Vec4 V, std::floating_point T = std::ranges::range_value_t<V>>
void testVector4_Constructor() {
  // Default
  V v0;
  EXPECT_EQ(v0[0], 0.0);
  EXPECT_EQ(v0[1], 0.0);
  EXPECT_EQ(v0[2], 0.0);
  EXPECT_EQ(v0[3], 0.0);

  // Scalar constructor
  V v1(1.0, 2.0, 3.0, 4.0);
  EXPECT_EQ(v1[0], 1.0);
  EXPECT_EQ(v1[1], 2.0);
  EXPECT_EQ(v1[2], 3.0);
  EXPECT_EQ(v1[3], 4.0);

  // Span constructor
  std::vector<T> vec = {2.0, 3.0, 4.0, 5.0, 6.0};
  V v2{std::span<T, 4>(vec)};
  EXPECT_EQ(v2[0], 2.0);
  EXPECT_EQ(v2[1], 3.0);
  EXPECT_EQ(v2[2], 4.0);
  EXPECT_EQ(v2[3], 5.0);

  // Initializer list assignment
  V v3 = {-1.0, 2.0, -3.0, -4.5};
  EXPECT_EQ(v3[0], -1.0);
  EXPECT_EQ(v3[1], 2.0);
  EXPECT_EQ(v3[2], -3.0);
  EXPECT_EQ(v3[3], -4.5);

  // Initializer list
  V v4 = {-1.5, -2.5, -3.5, -4.5};
  EXPECT_EQ(v4[0], -1.5);
  EXPECT_EQ(v4[1], -2.5);
  EXPECT_EQ(v4[2], -3.5);
  EXPECT_EQ(v4[3], -4.5);

  // Subspan
  V v5 = std::span(vec).template subspan<1, 4>();
  EXPECT_EQ(v5[0], 3.0);
  EXPECT_EQ(v5[1], 4.0);
  EXPECT_EQ(v5[2], 5.0);
  EXPECT_EQ(v5[3], vec.back());

  // First subspan
  V v6 = std::span(vec).template first<4>();
  EXPECT_EQ(v6[0], 2.0);
  EXPECT_EQ(v6[1], 3.0);
  EXPECT_EQ(v6[2], 4.0);
  EXPECT_EQ(v6[3], 5.0);

  // From Eigen
  Eigen::Vector4<T> eigen_v = {1.0, 2.0, 3.0, 4.0};
  V v7 = eigen_v;
  EXPECT_EQ(v7[0], 1.0);  
  EXPECT_EQ(v7[1], 2.0);
  EXPECT_EQ(v7[2], 3.0);
  EXPECT_EQ(v7[3], 4.0);
}

template <Vec3 V, typename T = typename V::Scalar>
void testVector3_Initializers() {
  // Unit vectors
  V e0 = UnitX<V>();
  EXPECT_EQ(e0[0], 1.0);
  EXPECT_EQ(e0[1], 0.0);
  EXPECT_EQ(e0[2], 0.0);

  V e1 = UnitY<V>();
  EXPECT_EQ(e1[0], 0.0);
  EXPECT_EQ(e1[1], 1.0);
  EXPECT_EQ(e1[2], 0.0);

  V e2 = UnitZ<V>();
  EXPECT_EQ(e2[0], 0.0);
  EXPECT_EQ(e2[1], 0.0);
  EXPECT_EQ(e2[2], 1.0);

  e0 = Unit<V, 0>();
  EXPECT_EQ(e0[0], 1.0);
  EXPECT_EQ(e0[1], 0.0);
  EXPECT_EQ(e0[2], 0.0);

  e1 = Unit<V, 1>();
  EXPECT_EQ(e1[0], 0.0);
  EXPECT_EQ(e1[1], 1.0);
  EXPECT_EQ(e1[2], 0.0);

  e2 = Unit<V, 2>();
  EXPECT_EQ(e2[0], 0.0);
  EXPECT_EQ(e2[1], 0.0);
  EXPECT_EQ(e2[2], 1.0);

  // Special vectors
  V v0 = Zero<V>();
  EXPECT_EQ(v0[0], 0.0);
  EXPECT_EQ(v0[1], 0.0);
  EXPECT_EQ(v0[2], 0.0);

  V v1 = Ones<V>();
  EXPECT_EQ(v1[0], 1.0);
  EXPECT_EQ(v1[1], 1.0);
  EXPECT_EQ(v1[2], 1.0);

  V v2 = Constant<V, T>(2.0);
  EXPECT_EQ(v2[0], 2.0);
  EXPECT_EQ(v2[1], 2.0);
  EXPECT_EQ(v2[2], 2.0);

  // Sequences
  // NOTE: implicit type conversion isn't allowed.
  V v3 = Sequence<V, T>(1, 2);
  EXPECT_EQ(v3[0], 1.0);
  EXPECT_EQ(v3[1], 3.0);
  EXPECT_EQ(v3[2], 5.0);

  V v4 = Sequence<V>(T(1), T(-1));
  EXPECT_EQ(v4[0], 1.0);
  EXPECT_EQ(v4[1], 0.0);
  EXPECT_EQ(v4[2], -1.0);

  // Test uniform distribution is always between 0 and 1
  std::mt19937 mt;
  std::set<T> samples;
  int num_samples = 1000;
  for (int i = 0; i < num_samples; ++i) {
    V v_rand = rand<V>(mt);
    for (auto el : v_rand) {
      EXPECT_GE(el, 0.0);
      EXPECT_LE(el, 1.0);
      samples.insert(el);
    }
  }
  EXPECT_EQ(samples.size(), num_samples * 3);

  // Test normal distribution has a mean of 0
  T sum = 0;
  for (int i = 0; i < num_samples; ++i) {
    V v_rand = randn<V>(mt);
    for (auto el : v_rand) {
      sum += el;
    }
  }
  EXPECT_NEAR(sum / (num_samples * 3), 0.0, 1e-1);
}

template <Vec4 V, typename T = typename V::Scalar>
void testVector4_Initializers() {
  // Unit vectors
  V e0 = UnitX<V>();
  EXPECT_EQ(e0[0], 1.0);
  EXPECT_EQ(e0[1], 0.0);
  EXPECT_EQ(e0[2], 0.0);
  EXPECT_EQ(e0[3], 0.0);

  V e1 = UnitY<V>();
  EXPECT_EQ(e1[0], 0.0);
  EXPECT_EQ(e1[1], 1.0);
  EXPECT_EQ(e1[2], 0.0);
  EXPECT_EQ(e1[3], 0.0);

  V e2 = UnitZ<V>();
  EXPECT_EQ(e2[0], 0.0);
  EXPECT_EQ(e2[1], 0.0);
  EXPECT_EQ(e2[2], 1.0);
  EXPECT_EQ(e2[3], 0.0);

  V e3 = Unit<V, 3>();
  EXPECT_EQ(e3[0], 0.0);
  EXPECT_EQ(e3[1], 0.0);
  EXPECT_EQ(e3[2], 0.0);
  EXPECT_EQ(e3[3], 1.0);

  e0 = UnitX<V>();
  EXPECT_EQ(e0[0], 1.0);
  EXPECT_EQ(e0[1], 0.0);
  EXPECT_EQ(e0[2], 0.0);
  EXPECT_EQ(e0[3], 0.0);

  e1 = UnitY<V>();
  EXPECT_EQ(e1[0], 0.0);
  EXPECT_EQ(e1[1], 1.0);
  EXPECT_EQ(e1[2], 0.0);
  EXPECT_EQ(e1[3], 0.0);

  e2 = UnitZ<V>();
  EXPECT_EQ(e2[0], 0.0);
  EXPECT_EQ(e2[1], 0.0);  
  EXPECT_EQ(e2[2], 1.0);
  EXPECT_EQ(e2[3], 0.0);

  e3 = Unit<V, 3>();
  EXPECT_EQ(e3[0], 0.0);
  EXPECT_EQ(e3[1], 0.0);
  EXPECT_EQ(e3[2], 0.0);
  EXPECT_EQ(e3[3], 1.0);

  // Special vectors
  V v0 = Zero<V>();
  EXPECT_EQ(v0[0], 0.0);
  EXPECT_EQ(v0[1], 0.0);
  EXPECT_EQ(v0[2], 0.0);
  EXPECT_EQ(v0[3], 0.0);

  V v1 = Ones<V>();
  EXPECT_EQ(v1[0], 1.0);
  EXPECT_EQ(v1[1], 1.0);
  EXPECT_EQ(v1[2], 1.0);
  EXPECT_EQ(v1[3], 1.0);

  V v2 = Constant<V, T>(2.0);
  EXPECT_EQ(v2[0], 2.0);
  EXPECT_EQ(v2[1], 2.0);
  EXPECT_EQ(v2[2], 2.0);
  EXPECT_EQ(v2[3], 2.0);

  // Sequences
  // NOTE: implicit type conversion isn't allowed.
  V v3 = Sequence<V, T>(1, 2);
  EXPECT_EQ(v3[0], 1.0);
  EXPECT_EQ(v3[1], 3.0);
  EXPECT_EQ(v3[2], 5.0);
  EXPECT_EQ(v3[3], 7.0);

  V v4 = Sequence<V>(T(1), T(-1));
  EXPECT_EQ(v4[0], 1.0);
  EXPECT_EQ(v4[1], 0.0);
  EXPECT_EQ(v4[2], -1.0);
  EXPECT_EQ(v4[3], -2.0);
}

template <Vec3 V, typename T = typename V::Scalar>
void testVector3_Setters() {
  // Unit vectors
  V v;
  setUnitX(v);
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 0.0);

  setUnitY(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 1.0);
  EXPECT_EQ(v[2], 0.0);

  setUnitZ(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 1.0);

  // Special vectors
  setZero(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 0.0);

  setOnes(v);
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 1.0);
  EXPECT_EQ(v[2], 1.0);

  setConstant(v, T(2.0));
  EXPECT_EQ(v[0], 2.0);
  EXPECT_EQ(v[1], 2.0);
  EXPECT_EQ(v[2], 2.0);

  // Direct value setting
  setValues(v, std::make_tuple(T(1.0), T(2.0), T(3.0)));
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 2.0);
  EXPECT_EQ(v[2], 3.0);
}

template <Vec4 V, typename T = typename V::Scalar>
void testVector4_Setters() {
  // Unit vectors
  V v;
  setUnitX(v);
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 0.0);
  EXPECT_EQ(v[3], 0.0);

  setUnitY(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 1.0);
  EXPECT_EQ(v[2], 0.0);
  EXPECT_EQ(v[3], 0.0);

  setUnitZ(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 1.0);
  EXPECT_EQ(v[3], 0.0);

  setUnitW(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 0.0);
  EXPECT_EQ(v[3], 1.0);

  // Special vectors
  setZero(v);
  EXPECT_EQ(v[0], 0.0);
  EXPECT_EQ(v[1], 0.0);
  EXPECT_EQ(v[2], 0.0);
  EXPECT_EQ(v[3], 0.0);

  setOnes(v);
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 1.0);
  EXPECT_EQ(v[2], 1.0);
  EXPECT_EQ(v[3], 1.0);

  setConstant(v, T(2.5));
  EXPECT_EQ(v[0], T(2.5));
  EXPECT_EQ(v[1], T(2.5));
  EXPECT_EQ(v[2], T(2.5));
  EXPECT_EQ(v[3], T(2.5));

  // Direct value setting
  setValues(v, std::make_tuple(T(1.0), T(2.0), T(3.0), T(4.0)));
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 2.0);
  EXPECT_EQ(v[2], 3.0); 
  EXPECT_EQ(v[3], 4.0);
}

template <Vec3 V, typename T = typename V::Scalar>
void testVector3_Arithmetic() {
  // Create test vectors
  V a{T(1), T(2), T(3)};
  V b{T(-1), T(0), T(2)};
  V c{T(2), T(-1), T(4)};
  T s1 = T(2);
  T s2 = T(3);

  // Vector addition
  // Commutativity: a + b = b + a
  EXPECT_EQ(a + b, b + a);

  // Associativity: (a + b) + c = a + (b + c)
  EXPECT_EQ((a + b) + c, a + (b + c));

  // Additive identity: a + 0 = a
  EXPECT_EQ(a + Zero<V>(), a);

  // Additive inverse: a + (-a) = 0
  EXPECT_EQ(a + (-a), Zero<V>());

  // Scalar multiplication
  // Associativity: (s1 * s2) * a = s1 * (s2 * a)
  EXPECT_EQ((s1 * s2) * a, s1 * (s2 * a));

  // Distributivity over vector addition: s1 * (a + b) = s1 * a + s1 * b
  EXPECT_EQ(s1 * (a + b), s1 * a + s1 * b);

  // Distributivity over scalar addition: (s1 + s2) * a = s1 * a + s2 * a
  EXPECT_EQ((s1 + s2) * a, s1 * a + s2 * a);

  // Scalar multiplication commutativity: s1 * a = a * s1
  EXPECT_EQ(s1 * a, a * s1);

  // Scalar multiplication identity: 1 * a = a
  EXPECT_EQ(T(1) * a, a);

  // Vector subtraction
  // a - b = a + (-b)
  EXPECT_EQ(a - b, a + (-b));

  // Scalar division
  // (a / s1) * s1 = a  (for s1 != 0)
  EXPECT_EQ((a / s1) * s1, a);

  // Basic vector operations
  V d = a + b;  // Addition
  EXPECT_EQ(d[0], a[0] + b[0]);
  EXPECT_EQ(d[1], a[1] + b[1]);
  EXPECT_EQ(d[2], a[2] + b[2]);

  V e = a - b;  // Subtraction
  EXPECT_EQ(e[0], a[0] - b[0]);
  EXPECT_EQ(e[1], a[1] - b[1]);
  EXPECT_EQ(e[2], a[2] - b[2]);

  V f = s1 * a;  // Scalar multiplication
  EXPECT_EQ(f[0], s1 * a[0]);
  EXPECT_EQ(f[1], s1 * a[1]);
  EXPECT_EQ(f[2], s1 * a[2]);

  V g = a / s1;  // Scalar division
  EXPECT_EQ(g[0], a[0] / s1);
  EXPECT_EQ(g[1], a[1] / s1);
  EXPECT_EQ(g[2], a[2] / s1);

  // Vector/Vector compound assignment operations
  {
    V h = a;
    h += b;  // Addition assignment
    EXPECT_EQ(h, a + b);
    EXPECT_EQ(h[0], a[0] + b[0]);
    EXPECT_EQ(h[1], a[1] + b[1]);
    EXPECT_EQ(h[2], a[2] + b[2]);

    h = a;
    h -= b;  // Subtraction assignment
    EXPECT_EQ(h, a - b);
    EXPECT_EQ(h[0], a[0] - b[0]);
    EXPECT_EQ(h[1], a[1] - b[1]);
    EXPECT_EQ(h[2], a[2] - b[2]);

    // Verify that compound assignments modify the original vector
    h = a;
    V h_orig = h;
    h += b;
    EXPECT_NE(h, h_orig);
    EXPECT_EQ(h[0], h_orig[0] + b[0]);
    EXPECT_EQ(h[1], h_orig[1] + b[1]);
    EXPECT_EQ(h[2], h_orig[2] + b[2]);
  }

  // Vector/Scalar compound assignment operations
  {
    V h = a;
    h *= s1;  // Multiplication assignment
    EXPECT_EQ(h, a * s1);
    EXPECT_EQ(h, s1 * a);  // Verify commutativity
    EXPECT_EQ(h[0], a[0] * s1);
    EXPECT_EQ(h[1], a[1] * s1);
    EXPECT_EQ(h[2], a[2] * s1);

    h = a;
    h /= s1;  // Division assignment
    EXPECT_EQ(h, a / s1);
    EXPECT_EQ(h[0], a[0] / s1);
    EXPECT_EQ(h[1], a[1] / s1);
    EXPECT_EQ(h[2], a[2] / s1);

    // Verify operations with different scalar types
    h = a;
    h *= T(2);  // Integer literal
    EXPECT_EQ(h, a * T(2));

    h = a;
    h /= T(2);  // Integer literal
    EXPECT_EQ(h, a / T(2));

    // Verify that compound assignments modify the original vector
    h = a;
    V h_orig = h;
    h *= s1;
    EXPECT_NE(h, h_orig);
    EXPECT_EQ(h[0], h_orig[0] * s1);
    EXPECT_EQ(h[1], h_orig[1] * s1);
    EXPECT_EQ(h[2], h_orig[2] * s1);
  }

  // Mixed operations
  {
    V h = a;
    h += b;
    h *= s1;
    EXPECT_EQ(h, (a + b) * s1);

    h = a;
    h *= s1;
    h += b;
    EXPECT_EQ(h, a * s1 + b);  // Different from above due to order

    // Chained operations
    h = a;
    (h += b) *= s1;
    EXPECT_EQ(h, (a + b) * s1);
  }
}

template <typename V, typename T = typename V::Scalar>
void testVector3_VectorOps() {
  // Test vectors
  V a{T(1), T(2), T(3)};   // norm = sqrt(14)
  V b{T(-2), T(1), T(2)};  // norm = 3
  V ex{T(1), T(0), T(0)};  // unit vector along x-axis
  V ey{T(0), T(1), T(0)};  // unit vector along y-axis

  // Sum
  EXPECT_EQ(sum(a), T(6));   // 1 + 2 + 3
  EXPECT_EQ(sum(b), T(1));   // -2 + 1 + 2
  EXPECT_EQ(sum(ex), T(1));  // 1 + 0 + 0

  // Get machine epsilon for type T
  const T eps = std::numeric_limits<T>::epsilon();

  // Norm and normSquared
  EXPECT_NEAR(normSquared(a), T(14), eps);  // 1^2 + 2^2 + 3^2
  EXPECT_NEAR(norm(a), std::sqrt(T(14)), eps);
  EXPECT_NEAR(normSquared(b), T(9), eps);  // (-2)^2 + 1^2 + 2^2
  EXPECT_NEAR(norm(b), T(3), eps);
  EXPECT_NEAR(normSquared(ex), T(1), eps);  // Unit vector
  EXPECT_NEAR(norm(ex), T(1), eps);

  // Dot product
  EXPECT_NEAR(dot(a, b), T(-2 + 2 + 6), eps);  // 1*(-2) + 2*1 + 3*2
  EXPECT_NEAR(dot(ex, ey), T(0), eps);         // Orthogonal unit vectors
  EXPECT_NEAR(dot(ex, ex), T(1), eps);         // Unit vector self dot

  // Angle between vectors
  EXPECT_NEAR(angleBetween(ex, ey), std::numbers::pi_v<T> / T(2),
              eps);                              // 90 degrees
  EXPECT_NEAR(angleBetween(ex, ex), T(0), eps);  // Same vector

  // Test vector normalization
  V a_norm = normalize(a);
  EXPECT_NEAR(norm(a_norm), T(1), eps);
  EXPECT_NEAR(dot(a, a_norm), norm(a),
              2 * eps);  // Original and normalized are parallel

  // Cross product
  V ez = cross(ex, ey);  // Should be unit vector in z direction
  EXPECT_NEAR(ez[0], T(0), eps);
  EXPECT_NEAR(ez[1], T(0), eps);
  EXPECT_NEAR(ez[2], T(1), eps);

  // Cross product properties
  V cab = cross(a, b);
  V cba = cross(b, a);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(cab[i], -cba[i], eps);  // Anti-commutative
  }

  // Cross product with self should be zero
  V self_cross = cross(a, a);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(self_cross[i], T(0), eps);
  }

  // Distance
  EXPECT_NEAR(distance(a, b), norm(a - b), eps);
  EXPECT_NEAR(distance(a, a), T(0), eps);
  EXPECT_NEAR(distance(ez, ey), std::sqrt(T(2)), eps);
  EXPECT_NEAR(distance(ez, UnitX<V>()), std::sqrt(T(2)), eps);
}

/////////////////////////////////////////////
/// generic::Vector3 Tests
/////////////////////////////////////////////

namespace generic {

TEST(VectorTests, Vector3_Concepts) {
  testVector3_Concepts<Vector3<float>>();
  testVector3_Concepts<Vector3<double>>();
}

TEST(VectorTests, Vector3_Constructor) {
  testVector3_Constructor<Vector3<float>, float>();
  testVector3_Constructor<Vector3<double>, double>();
}

TEST(VectorTests, Vector3_Initializers) {
  testVector3_Initializers<Vector3<float>>();
  testVector3_Initializers<Vector3<double>>();
}

TEST(VectorTests, Vector3_Setters) {
  testVector3_Setters<Vector3<float>>();
  testVector3_Setters<Vector3<double>>();
}

TEST(VectorTests, Vector3_Arithmetic) {
  testVector3_Arithmetic<Vector3<float>>();
  testVector3_Arithmetic<Vector3<double>>();
}

TEST(VectorTests, Vector3_VectorOps) {
  testVector3_VectorOps<Vector3<float>>();
  testVector3_VectorOps<Vector3<double>>();
}

TEST(VectorTests, Vector4_Concepts) {
  testVector3_Concepts<Vector3<float>>();
  testVector3_Concepts<Vector3<double>>();
}

TEST(VectorTests, Vector4_Constructor) {
  testVector3_Constructor<Vector3<float>, float>();
  testVector3_Constructor<Vector3<double>, double>();
}

TEST(VectorTests, Vector4_Initializers) {
  testVector3_Initializers<Vector3<float>>();
  testVector3_Initializers<Vector3<double>>();
}

TEST(VectorTests, Vector4_Setters) {
  testVector3_Setters<Vector3<float>>();
  testVector3_Setters<Vector3<double>>();
}

}  // namespace generic

TEST(VectorTests, MatGroups) {
  EXPECT_TRUE((
      std::same_as<MatGroup<Generic, double>::Vec3, generic::Vector3<double>>));
  EXPECT_TRUE((
      std::same_as<MatGroup<Generic, double>::Vec4, generic::Vector4<double>>));

  EXPECT_FALSE((
      std::same_as<MatGroup<SIMD, double>::Vec3, generic::Vector3<double>>));
  EXPECT_TRUE((
      std::same_as<MatGroup<SIMD, double>::Vec3, simd::Vector3<double>>));
  EXPECT_TRUE((
      std::same_as<MatGroup<SIMD, double>::Vec4, generic::Vector4<double>>));
}

}  // namespace se3