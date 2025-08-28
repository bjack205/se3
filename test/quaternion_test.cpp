#include "se3/attitude/quaternion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <string>

#include "se3/attitude/generic/quaternion.hpp"
#include "se3/attitude/quaternion.hpp"
#include "se3/linear_algebra/arrays/vectors_arrays.hpp"
#include "se3/linear_algebra/matrix_ops.hpp"
#include "se3/linear_algebra/vector_ops.hpp"

namespace se3 {
template <AbstractQuaternion Quaternion>
void TestQuaternionConcepts() {
  EXPECT_TRUE(std::ranges::random_access_range<Quaternion>);
  EXPECT_TRUE(Vec4<Quaternion>);
  EXPECT_TRUE(AbstractQuaternion<Quaternion>);
  EXPECT_TRUE(std::is_trivially_copyable_v<Quaternion>);
  EXPECT_TRUE(std::is_trivially_move_assignable_v<Quaternion>);
  EXPECT_TRUE(std::regular<Quaternion>);
}

template <AbstractQuaternion Quaternion>
void TestQuaternionVectorInterface() {
  using T = std::ranges::range_value_t<Quaternion>;
  Quaternion q;
  q.w = 0;
  EXPECT_DOUBLE_EQ(q[0], 0.0);
  for (auto el : q) {
    EXPECT_DOUBLE_EQ(el, 0.0);
  }
  std::ranges::all_of(q, [](auto el) { return el == 0.0; });
  EXPECT_TRUE((std::same_as<std::ranges::range_value_t<Quaternion>, T>));

  setConstant(q, 1.0);
  for (auto el : q) {
    EXPECT_DOUBLE_EQ(el, 1.0);
  }
}

template <AbstractQuaternion Q>
void TestQuaternionNorm() {
  using T = std::ranges::range_value_t<Q>;
  const T tol = 2 * std::numeric_limits<T>::epsilon();

  Q q = identity<Q>();
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

  // TODO: loop over random pairs.
  Q q1 = Q(1.0, 2.0, 3.0, 4.0);
  Q q2 = Q(5.0, 6.0, 7.0, 8.0);
  Q q3 = q1 + q2;

  // Check sum is correct
  EXPECT_NEAR(q3.x, 6.0, tol);
  EXPECT_NEAR(q3.y, 8.0, tol);
  EXPECT_NEAR(q3.z, 10.0, tol);
  EXPECT_NEAR(q3.w, 12.0, tol);

  // Check sum is commutative
  EXPECT_NEAR((q1 + q2).x, (q2 + q1).x, tol);
  EXPECT_NEAR((q1 + q2).y, (q2 + q1).y, tol);
  EXPECT_NEAR((q1 + q2).z, (q2 + q1).z, tol);
  EXPECT_NEAR((q1 + q2).w, (q2 + q1).w, tol);

  // Check sum is associative
  EXPECT_NEAR((q1 + q2 + q3).x, (q1 + (q2 + q3)).x, tol);
  EXPECT_NEAR((q1 + q2 + q3).y, (q1 + (q2 + q3)).y, tol);
  EXPECT_NEAR((q1 + q2 + q3).z, (q1 + (q2 + q3)).z, tol);
  EXPECT_NEAR((q1 + q2 + q3).w, (q1 + (q2 + q3)).w, tol);
}

template <AbstractQuaternion Q>
void TestAngleBetween() {
  using T = std::ranges::range_value_t<Q>;
  using se3::angleBetween;
  const double tol = 2 * std::numeric_limits<T>::epsilon();

  // Identical quaternions (angle should be 0)
  Q q1 = identity<Q>();
  Q q2 = identity<Q>();
  EXPECT_NEAR(angleBetween(q1, q2), 0.0, tol);

  // Opposite quaternions
  EXPECT_NEAR(angleBetween(q1, -q1), 0.0, tol);

  // 90 degree rotation about x axis
  double theta = std::numbers::pi / 2;
  q1 = Q(std::sin(theta / 2), 0, 0, std::cos(theta / 2));  // 90 deg
  q2 = identity<Q>();
  EXPECT_NEAR(angleBetween(q1, q2), theta, tol);

  // 180 degree rotation about y axis
  theta = std::numbers::pi;
  q1 = Q(0, std::sin(theta / 2), 0, std::cos(theta / 2));  // 180 deg
  q2 = identity<Q>();
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
  Q q1 = Q(std::sin(0.1), 0, 0, std::cos(0.1));
  Q q2 = Q(std::sin(0.2), 0, 0, std::cos(0.2));
  Q q3 = Q(std::sin(0.3), 0, 0, std::cos(0.3));
  EXPECT_LT(angleBetween(q1 * q2, q3), tol);

  q1 = Q(0, std::sin(0.1), 0, std::cos(0.1));
  q2 = Q(0, std::sin(-0.2), 0, std::cos(-0.2));
  q3 = Q(0, std::sin(-0.1), 0, std::cos(-0.1));
  EXPECT_LT(angleBetween(q1 * q2, q3), tol);

  q1 = Q(std::sin(-0.1), 0, 0, std::cos(-0.1));
  q2 = Q(std::sin(-0.2), 0, 0, std::cos(-0.2));
  q3 = Q(std::sin(-0.3), 0, 0, std::cos(-0.3));
  EXPECT_LT(angleBetween(q1 * q2, q3), tol);

  // TODO: do these with random quaternions.
  // Check identity
  q1 = Q(-0.5509423119979514, 0.639064294304671, -0.3007625682094623,
         0.4445236485939453);
  EXPECT_LT(angleBetween(q1 * identity<Q>(), q1), tol);
  EXPECT_LT(angleBetween(identity<Q>() * q1, q1), tol);

  // Check conjugate
  q1 = Q(1, 2, 3, 0);
  q2 = Q(-1, -2, -3, 0);
  EXPECT_LT(norm(conjugate(q1) - q2), tol);

  // Check inverse
  q1 = Q(1, 2, 3, -1);
  EXPECT_NEAR(normSquared(q1), 15.0, tol);
  q2 = Q(-1.0 / 15.0, -2.0 / 15.0, -3.0 / 15.0, -1.0 / 15.0);
  EXPECT_LT(norm(inverse(q1) - q2), tol);

  // Check multiplication by inverse
  q1 = Q(-1, 1, 2, 3);
  EXPECT_LT(angleBetween(q1 * inverse(q1), identity<Q>()), tol);
  EXPECT_LT(angleBetween(inverse(q1) * q1, identity<Q>()), tol);

  // Check associativity
  q1 = Q(1, 2, 3, 4);
  q2 = Q(5, 6, 7, 8);
  q3 = Q(9, 10, 11, 12);
  EXPECT_LT(angleBetween((q1 * q2) * q3, q1 * (q2 * q3)), tol);
}

template <AbstractQuaternion Q>
void TestVectorRotation() {
  using T = std::ranges::range_value_t<Q>;
  using V = generic::Vector3<T>;
  const T tol = 2 * std::numeric_limits<T>::epsilon();

  // 90-degree rotation about z-axis
  T theta = std::numbers::pi / 2;
  Q qz = Q(0, 0, std::sin(theta / 2), std::cos(theta / 2));
  V v{1, 2, 0};
  V v_rot = qz * v;
  EXPECT_NEAR(v_rot[0], -v[1], tol);
  EXPECT_NEAR(v_rot[1], v[0], tol);
  EXPECT_NEAR(v_rot[2], 0.0, tol);

  // 180-degree rotation about y-axis
  theta = std::numbers::pi;
  Q qy = Q(0, std::sin(theta / 2), 0, std::cos(theta / 2));
  v = V{1, 0, 0};
  v_rot = qy * v;
  EXPECT_NEAR(v_rot[0], -1.0, tol);
  EXPECT_NEAR(v_rot[1], 0.0, tol);
  EXPECT_NEAR(v_rot[2], 0.0, tol);

  // Identity rotation
  v = V{1, 2, 3};
  EXPECT_EQ(identity<Q>() * v, v);

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
    Q q = expm(v0);
    EXPECT_LT(angleBetween(expm(v0), identity<Q>()), tol);
  }

  // Logm(identity) == 0
  {
    V v = logm(identity<Q>());
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

    // Test quaternion exponent and logarithm for unit and non-unit
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

template <AbstractQuaternion Q>
void TestRotationMatrix() {
  using Mat3 = MatrixGroupFor<Q>::Mat3;
  using Vec3 = MatrixGroupFor<Q>::Vec3;
  using Scalar = std::ranges::range_value_t<Q>;
  Scalar tol = std::numeric_limits<Scalar>::epsilon() * 10;

  auto check_orthogonality = [tol](const auto& m) {
    EXPECT_LT(norm(transpose(m) * m - UniformScaling<Scalar>()), tol);
    EXPECT_NEAR(determinant(m), 1, tol);
  };
  auto q1 = Q(std::sin(0.1), 0, 0, std::cos(0.1));
  auto q2 = Q(0, std::sin(-0.2), 0, std::cos(-0.2));
  auto q3 = Q(0, 0, std::sin(-0.1), std::cos(-0.1));

  Vec3 v1{1, 2, 3};
  Mat3 m1 = rotationMatrix(q1);
  check_orthogonality(m1);
  EXPECT_LT(norm(m1 * v1 - q1 * v1), tol);
  EXPECT_LT(norm(Transpose(m1) * v1 - inverse(q1) * v1), tol);

  Mat3 m2 = rotationMatrix(q2);
  check_orthogonality(m2);
  EXPECT_LT(norm(m2 * v1 - q2 * v1), tol);
  EXPECT_LT(norm(Transpose(m2) * v1 - inverse(q2) * v1), tol);

  Mat3 m3 = rotationMatrix(q3);
  check_orthogonality(m3);
  EXPECT_LT(norm(m3 * v1 - q3 * v1), tol);
  EXPECT_LT(norm(Transpose(m3) * v1 - inverse(q3) * v1), tol);
}

template <AbstractQuaternion Q>
void TestQuatMats() {
  using namespace quatmats;
  using Scalar = std::ranges::range_value_t<Q>;
  Scalar tol = std::numeric_limits<Scalar>::epsilon() * 10;
  using Mat4 = MatrixGroupFor<Q>::Mat4;

  std::mt19937 gen(0);
  std::normal_distribution dist(0.0);
  auto q0 = normalize(Q(dist(gen), dist(gen), dist(gen), dist(gen)));
  auto q1 = normalize(Q(dist(gen), dist(gen), dist(gen), dist(gen)));

  EXPECT_LT(angleBetween(q0 * q1, L(q0) * q1), tol);
  EXPECT_LT(angleBetween(q0 * q1, R(q1) * q0), tol);
  EXPECT_LT(angleBetween(inverse(q0) * q1, Transpose(L(q0)) * q1), tol);
  EXPECT_LT(angleBetween(q0 * inverse(q1), Transpose(R(q1)) * q0), tol);
  EXPECT_LT(angleBetween(inverse(q0), T<Mat4>() * q0), tol);
  EXPECT_LT(angleBetween(q0 * inverse(q1), L(q0) * (T<Mat4>() * q1)), tol);
}

template <AbstractQuaternion Q>
void TestInitializers() {
  using T = std::ranges::range_value_t<Q>;
  const T tol = std::numeric_limits<T>::epsilon();

  std::mt19937 gen(0);
  std::uniform_real_distribution<T> dist;
  for (int i = 0; i < 100; ++i) {
    Q q_rand = randomRotation<Q>(gen);
    EXPECT_NEAR(norm(q_rand), 1.0, tol);

    T angle = dist(gen) * 2 * std::numbers::pi;
    Q qx = RotX<Q>(angle);
    EXPECT_NEAR(norm(qx), 1.0, tol);
    EXPECT_NEAR(qx.x, std::sin(angle / 2), tol);
    EXPECT_NEAR(qx.y, 0.0, tol);
    EXPECT_NEAR(qx.z, 0.0, tol);
    EXPECT_NEAR(qx.w, std::cos(angle / 2), tol);

    Q qy = RotY<Q>(angle);
    EXPECT_NEAR(norm(qy), 1.0, tol);
    EXPECT_NEAR(qy.x, 0.0, tol);
    EXPECT_NEAR(qy.y, std::sin(angle / 2), tol);
    EXPECT_NEAR(qy.z, 0.0, tol);
    EXPECT_NEAR(qy.w, std::cos(angle / 2), tol);

    Q qz = RotZ<Q>(angle);
    EXPECT_NEAR(norm(qz), 1.0, tol);
    EXPECT_NEAR(qz.x, 0.0, tol);
    EXPECT_NEAR(qz.y, 0.0, tol);
    EXPECT_NEAR(qz.z, std::sin(angle / 2), tol);
    EXPECT_NEAR(qz.w, std::cos(angle / 2), tol);
  }
}

namespace generic {
TEST(QuaternionTests, generic_Concepts) {
  TestQuaternionConcepts<Quaternion<double>>();
  TestQuaternionConcepts<Quaternion<float>>();
}

TEST(QuaternionTests, generic_VectorInterface) {
  TestQuaternionVectorInterface<Quaternion<double>>();
  TestQuaternionVectorInterface<Quaternion<float>>();
}

TEST(QuaternionTests, generic_Norm) {
  TestQuaternionNorm<Quaternion<double>>();
  TestQuaternionNorm<Quaternion<float>>();
}

TEST(QuaternionTests, generic_Sum) {
  TestQuaternionSum<Quaternion<double>>();
  TestQuaternionSum<Quaternion<float>>();
}

TEST(QuaternionTests, generic_AngleBetween) {
  TestAngleBetween<Quaternion<double>>();
  TestAngleBetween<Quaternion<float>>();
}

TEST(QuaternionTests, generic_Composition) {
  TestQuaternionComposition<Quaternion<double>>();
  TestQuaternionComposition<Quaternion<float>>();
}

TEST(QuaternionTests, generic_VectorRotation) {
  TestVectorRotation<Quaternion<double>>();
  TestVectorRotation<Quaternion<float>>();
}

TEST(QuaternionTests, generic_ExponentialAndLogarithmMaps) {
  TestQuaternionExpLog<Quaternion<double>, Vector3<double>>();
  TestQuaternionExpLog<Quaternion<float>, Vector3<float>>();
}

TEST(QuaternionTests, generic_RotationMatrix) {
  TestRotationMatrix<Quaternion<double>>();
  TestRotationMatrix<Quaternion<float>>();
}

TEST(QuaternionTests, generic_QuatMats) {
  TestQuatMats<Quaternion<double>>();
  TestQuatMats<Quaternion<float>>();

  using Q = Quaternion<float>;
  std::mt19937 gen(0);
  std::normal_distribution dist(0.0);
  auto q0 = normalize(Q(dist(gen), dist(gen), dist(gen), dist(gen)));
  auto q1 = normalize(Q(dist(gen), dist(gen), dist(gen), dist(gen)));
  auto q2 = q0 * q1;
  auto q2_ = quatmats::L(q0) * q1;
  auto diff = q2 - q2_;
}

TEST(QuaternionTests, generic_Initializers) {
  TestInitializers<Quaternion<double>>();
  TestInitializers<Quaternion<float>>();
}
}  // namespace generic

TEST(ArrayVectors, Test) {
  static_assert(Vec3<arrays::Vector3<double>>);
  arrays::Vector3<double> v(3.0, 4.0, 0.0);
  double n2 = normSquared(v);
  EXPECT_NEAR(n2, 25, 1e-10);
}

}  // namespace se3