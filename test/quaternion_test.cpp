#include "se3/attitude/quaternion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <string>

#include "se3/attitude/quaternion_ops.hpp"
#include "se3/linear_algebra/generic/matrices_generic.hpp"
#include "se3/linear_algebra/transpose.hpp"

namespace se3 {

TEST(QuaternionTests, VectorInterface) {
  Quaternion<double> q;
  q.w = 0;
  EXPECT_DOUBLE_EQ(q[0], 0.0);
  for (auto el : q) {
    EXPECT_DOUBLE_EQ(el, 0.0);
  }
  std::ranges::all_of(q, [](auto el) { return el == 0.0; });
  EXPECT_TRUE(
      (std::same_as<std::ranges::range_value_t<Quaternion<double>>, double>));
}

TEST(QuaternionTests, Concepts) {
  EXPECT_TRUE(std::ranges::random_access_range<Quaternion<double>>);
  EXPECT_TRUE(AbstractVector4<Quaternion<double>>);
  EXPECT_TRUE(AbstractQuaternion<Quaternion<double>>);
}

template <AbstractQuaternion Q>
void TestQuaternionNorm() {
  using T = std::ranges::range_value_t<Q>;
  const T tol = std::numeric_limits<T>::epsilon();

  Q q = Q::Identity();
  EXPECT_NEAR(norm(q), 1.0, tol);
  EXPECT_NEAR(normSquared(q), 1.0, tol);

  q = Q(0.5, 0.5, 0.5, 0.5);
  EXPECT_NEAR(norm(q), 1.0, tol);
  EXPECT_NEAR(normSquared(q), 1.0, tol);

  q = Q(1.0, 2.0, 3.0, 4.0);
  EXPECT_NEAR(norm(q), std::sqrt(30.0), tol);
  EXPECT_NEAR(normSquared(q), 30.0, tol);

  q = normalize(Q(1.0, 2.0, 3.0, 4.0));
  EXPECT_NEAR(norm(q), 1.0, tol);
  EXPECT_NEAR(normSquared(q), 1.0, tol);
}

template <AbstractQuaternion Q>
void TestQuaternionSum() {
  using T = std::ranges::range_value_t<Q>;
  const T tol = std::numeric_limits<T>::epsilon();

  Q q1 = Q(1.0, 2.0, 3.0, 4.0);
  Q q2 = Q(5.0, 6.0, 7.0, 8.0);
  Q q3 = q1 + q2;

  // Check sum is correct
  EXPECT_NEAR(q3.w, 6.0, tol);
  EXPECT_NEAR(q3.x, 8.0, tol);
  EXPECT_NEAR(q3.y, 10.0, tol);
  EXPECT_NEAR(q3.z, 12.0, tol);

  // Check sum is commutative
  EXPECT_NEAR((q1 + q2).w, (q2 + q1).w, tol);
  EXPECT_NEAR((q1 + q2).x, (q2 + q1).x, tol);
  EXPECT_NEAR((q1 + q2).y, (q2 + q1).y, tol);
  EXPECT_NEAR((q1 + q2).z, (q2 + q1).z, tol);

  // Check sum is associative
  EXPECT_NEAR((q1 + q2 + q3).w, (q1 + (q2 + q3)).w, tol);
  EXPECT_NEAR((q1 + q2 + q3).x, (q1 + (q2 + q3)).x, tol);
  EXPECT_NEAR((q1 + q2 + q3).y, (q1 + (q2 + q3)).y, tol);
  EXPECT_NEAR((q1 + q2 + q3).z, (q1 + (q2 + q3)).z, tol);
}

TEST(QuaternionTests, Sum) {
  TestQuaternionSum<Quaternion<double>>();
  TestQuaternionSum<Quaternion<float>>();
}

TEST(QuaternionTests, AngleBetween) {
  using Q = Quaternion<double>;
  using se3::angleBetween;
  const double tol = 1e-12;

  // Identical quaternions (angle should be 0)
  Q q1 = Q::Identity();
  Q q2 = Q::Identity();
  EXPECT_NEAR(angleBetween(q1, q2), 0.0, tol);

  // Opposite quaternions
  q2 = Q(-1, 0, 0, 0);
  EXPECT_NEAR(angleBetween(q1, q2), 0.0, tol);

  // 90 degree rotation about x axis
  double theta = std::numbers::pi / 2;
  q1 = Q(std::cos(theta / 2), std::sin(theta / 2), 0, 0);  // 90 deg
  q2 = Q::Identity();
  EXPECT_NEAR(angleBetween(q1, q2), theta, tol);

  // 180 degree rotation about y axis
  theta = std::numbers::pi;
  q1 = Q(std::cos(theta / 2), 0, std::sin(theta / 2), 0);  // 180 deg
  q2 = Q::Identity();
  EXPECT_NEAR(angleBetween(q1, q2), theta, tol);
  EXPECT_NEAR(angleBetween(flipQuaternion(q1), q2), theta, tol);
  EXPECT_NEAR(angleBetween(q1, flipQuaternion(q2)), theta, tol);
  EXPECT_NEAR(angleBetween(q2, flipQuaternion(q1)), theta, tol);
  EXPECT_NEAR(angleBetween(flipQuaternion(q2), q1), theta, tol);

  // Arbitrary pair
  q1 = Q(0.70710678, 0.70710678, 0, 0);  // 90 deg about x
  q2 = Q(0.70710678, 0, 0.70710678, 0);  // 90 deg about y
  double ang = angleBetween(q1, q2);
  EXPECT_GT(ang, 0.0);
  EXPECT_LT(ang, std::numbers::pi);
}

template <AbstractQuaternion Q>
void TestQuaternionComposition() {
  using T = std::ranges::range_value_t<Q>;
  const T tol = std::numeric_limits<T>::epsilon();

  // Check rotations along x, y, z add
  Q q1 = Q(std::cos(0.1), std::sin(0.1), 0, 0);
  Q q2 = Q(std::cos(0.2), std::sin(0.2), 0, 0);
  Q q3 = Q(std::cos(0.3), std::sin(0.3), 0, 0);
  EXPECT_LT(angleBetween(q1 * q2, q3), tol);

  q1 = Q(std::cos(0.1), 0, std::sin(0.1), 0);
  q2 = Q(std::cos(-0.2), 0, std::sin(-0.2), 0);
  q3 = Q(std::cos(-0.1), 0, std::sin(-0.1), 0);
  EXPECT_LT(angleBetween(q1 * q2, q3), tol);

  q1 = Q(std::cos(-0.1), 0, 0, std::sin(-0.1));
  q2 = Q(std::cos(-0.2), 0, 0, std::sin(-0.2));
  q3 = Q(std::cos(-0.3), 0, 0, std::sin(-0.3));
  EXPECT_LT(angleBetween(q1 * q2, q3), tol);

  // Check identity
  q1 = Q(-0.5509423119979514, 0.639064294304671, -0.3007625682094623,
         0.4445236485939453);
  EXPECT_LT(angleBetween(q1 * Q::Identity(), q1), tol);
  EXPECT_LT(angleBetween(Q::Identity() * q1, q1), tol);

  // Check conjugate
  q1 = Q(0, 1, 2, 3);
  q2 = Q(0, -1, -2, -3);
  EXPECT_LT(norm(conjugate(q1) - q2), tol);

  // Check inverse
  q1 = Q(-1, 1, 2, 3);
  EXPECT_NEAR(normSquared(q1), 15.0, tol);
  q2 = Q(-1.0 / 15.0, -1.0 / 15.0, -2.0 / 15.0, -3.0 / 15.0);
  EXPECT_LT(norm(inverse(q1) - q2), tol);

  // Check multiplication by inverse
  q1 = Q(-1, 1, 2, 3);
  EXPECT_LT(angleBetween(q1 * inverse(q1), Q::Identity()), tol);
  EXPECT_LT(angleBetween(inverse(q1) * q1, Q::Identity()), tol);

  // Check associativity
  q1 = Q(1, 2, 3, 4);
  q2 = Q(5, 6, 7, 8);
  q3 = Q(9, 10, 11, 12);
  EXPECT_LT(angleBetween((q1 * q2) * q3, q1 * (q2 * q3)), tol);
}

TEST(QuaternionTests, Composition) {
  TestQuaternionComposition<Quaternion<float>>();
  TestQuaternionComposition<Quaternion<double>>();
}

TEST(QuaternionTests, RotateVector) {
  using Q = Quaternion<double>;
  using V = generic::Vector3<double>;
  const double tol = std::numeric_limits<double>::epsilon();

  // 90-degree rotation about z-axis
  double theta = std::numbers::pi / 2;
  Q qz = Q(std::cos(theta / 2), 0, 0, std::sin(theta / 2));
  V v{1, 0, 0};
  V v_rot = qz * v;
  EXPECT_NEAR(v_rot[0], 0.0, tol);
  EXPECT_NEAR(v_rot[1], 1.0, tol);
  EXPECT_NEAR(v_rot[2], 0.0, tol);

  // 180-degree rotation about y-axis
  theta = std::numbers::pi;
  Q qy = Q(std::cos(theta / 2), 0, std::sin(theta / 2), 0);
  v = V{1, 0, 0};
  v_rot = qy * v;
  EXPECT_NEAR(v_rot[0], -1.0, tol);
  EXPECT_NEAR(v_rot[1], 0.0, tol);
  EXPECT_NEAR(v_rot[2], 0.0, tol);

  // Identity rotation
  v = V{1, 2, 3};
  EXPECT_EQ(Q::Identity() * v, v);

  // Check rotation is the same as q x q(v) x q_c
  // NOTE: this is flipped for passive rotations
  Q v_rot_quat = qy * Q::Pure(v) * conjugate(qy);
  EXPECT_LT(norm(qy * v - quatvec(v_rot_quat)), tol);
  EXPECT_LT(norm(flipQuaternion(qy) * v - quatvec(v_rot_quat)), tol);
}

template <typename Q, typename V>
void TestQuaternionExpLog() {
  using se3::exp;
  using se3::expm;
  using se3::log;
  using se3::logm;
  using T = std::ranges::range_value_t<V>;
  const T tol = std::numeric_limits<T>::epsilon() * 10;

  // TODO: look into tightening up this tolerance
  const T tol_round_trip = std::pow(tol, 0.5);

  // Expm(0) == identity quaternion
  {
    V v0{0, 0, 0};
    EXPECT_LT(angleBetween(expm(v0), Q::Identity()), tol);
  }

  // Logm(identity) == 0
  {
    V v = logm(Q::Identity());
    EXPECT_NEAR(v[0], 0.0, tol);
    EXPECT_NEAR(v[1], 0.0, tol);
    EXPECT_NEAR(v[2], 0.0, tol);
  }

  std::mt19937 gen(0);
  std::normal_distribution<T> dist(0);
  int num_axes = 100;

  // Test round-trip for a range of magnitudes (including small-angle case)
  std::vector<T> mags = {T(1e-6),
                         T(1e-5),
                         T(0.01),
                         T(0.1),
                         T(0.5),
                         T(std::numbers::pi / 2),
                         T(std::numbers::pi),
                         T(2.0)};

  for (int i = 0; i < num_axes; ++i) {
    // Generate a random axis using a given seed
    auto axis = normalize(V{dist(gen), dist(gen), dist(gen)});

    // Test exponential and logarithmic maps
    for (T mag : mags) {
      V v = axis * mag;
      Q q = expm(axis * mag);

      EXPECT_NEAR(norm(q), 1.0, tol);
      T angle = std::acos(q.w) * 2;
      EXPECT_NEAR(angle, mag, std::sqrt(tol));

      EXPECT_LT(norm(v - logm(q)), tol) << "mag=" << mag;
      EXPECT_LT(angleBetween(q, expm(logm(q))), tol_round_trip)
          << "mag=" << mag;
    }

    // Test quaternion exponent and logarithm for unit and non-unit quaternions
    for (T mag : mags) {
      Q q_unit = expm(axis * mag);  // unit quaternion

      // Check Exp(Log(q)) ≈ q for unit q
      Q q_back = normalize(exp(log(q_unit)));
      EXPECT_LT(angleBetween(q_unit, q_back), tol_round_trip) << "mag=" << mag;

      // Check Log(Exp(q)) ≈ q for unit q
      q_back = normalize(log(exp(q_unit)));
      EXPECT_LT(angleBetween(q_unit, q_back), tol_round_trip) << "mag=" << mag;

      for (T mag2 : std::vector<T>{1e-6, 1e-5, 1e-3, 0.5, 0.9, 1.1, 1.5}) {
        Q q_non_unit = flipToPositiveScalar(q_unit * mag2);

        q_back = flipToPositiveScalar(exp(log(q_non_unit)));
        EXPECT_LT(norm(q_non_unit - q_back), tol_round_trip) << "mag=" << mag2;

        q_back = flipToPositiveScalar(log(exp(q_non_unit)));
        EXPECT_LT(norm(q_non_unit - q_back), tol_round_trip) << "mag=" << mag2;
      }
    }
  }
}

TEST(QuaternionTests, ExponentialAndLogarithmMaps) {
  TestQuaternionExpLog<Quaternion<double>, generic::Vector3<double>>();
  TestQuaternionExpLog<Quaternion<float>, generic::Vector3<float>>();
}

TEST(QuaternionTests, QuatMats) {
  using Q = Quaternion<double>;
  using namespace quatmats;

  std::mt19937 gen(0);
  std::normal_distribution dist(0.0);
  auto q0 = normalize(Q(dist(gen), dist(gen), dist(gen), dist(gen)));
  auto q1 = normalize(Q(dist(gen), dist(gen), dist(gen), dist(gen)));

  EXPECT_LT(angleBetween(q0 * q1, L(q0) * q1), 1e-10);
  EXPECT_LT(angleBetween(q0 * q1, R(q1) * q0), 1e-10);
  EXPECT_LT(angleBetween(inverse(q0) * q1, Transpose(L(q0)) * q1), 1e-10);
  EXPECT_LT(angleBetween(q0 * inverse(q1), Transpose(R(q1)) * q0), 1e-10);
  EXPECT_LT(angleBetween(inverse(q0), T() * q0), 1e-10);
  EXPECT_LT(angleBetween(q0 * inverse(q1), L(q0) * (T() * q1)), 1e-10);
}

}  // namespace se3