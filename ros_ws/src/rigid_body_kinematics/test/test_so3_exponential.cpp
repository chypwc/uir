#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/rotation3.hpp"

TEST(So3ExponentialTest, ZeroRotationVectorProducesExactIdentity)
{
  const Eigen::Vector3d rotation_vector = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(rotation_vector);

  const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d actual = rotation.matrix();

  EXPECT_TRUE((actual.array() == expected.array()).all());
}

TEST(So3ExponentialTest, QuarterTurnAboutXMatchesAnalyticRotation)
{
  const double half_pi = 0.5 * std::numbers::pi;
  const Eigen::Vector3d rotation_vector(half_pi, 0.0, 0.0);

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(rotation_vector);

  Eigen::Matrix3d expected;
  // clang-format off
  expected <<
    1.0, 0.0,  0.0,
    0.0, 0.0, -1.0,
    0.0, 1.0,  0.0;
  // clang-format on

  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(rotation.matrix().isApprox(expected, tolerance));
}

TEST(So3ExponentialTest, SmallNonzeroRotationIsNotCollapsedToIdentity)
{
  constexpr double angle = 1.0e-8;  // radians

  const Eigen::Vector3d rotation_vector(0.0, 0.0, angle);

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(rotation_vector);

  Eigen::Matrix3d expected;

  // Independent analytic rotation about the positive z-axis.
  // clang-format off
  expected <<
    std::cos(angle), -std::sin(angle), 0.0,
    std::sin(angle),  std::cos(angle), 0.0,
    0.0,              0.0,             1.0;
  // clang-format on

  const Eigen::Matrix3d actual = rotation.matrix();

  constexpr double tolerance = 1.0e-15;

  EXPECT_TRUE(actual.isApprox(expected, tolerance));

  // These checks specifically ensure that the nonzero rotation was
  // not classified as the exact identity.
  EXPECT_NE(actual(0, 1), 0.0);
  EXPECT_NE(actual(1, 0), 0.0);
}

TEST(So3ExponentialTest, MaximumSupportedAngleIsAccepted)
{
  const rigid_body_kinematics::NumericalPolicy policy;
  const double angle = policy.maximum_exponential_angle;

  const Eigen::Vector3d rotation_vector(angle, 0.0, 0.0);

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(
      rotation_vector, policy);

  Eigen::Matrix3d expected;

  // clang-format off
  expected <<
    1.0, 0.0,             0.0,
    0.0, std::cos(angle), -std::sin(angle),
    0.0, std::sin(angle),  std::cos(angle);
  // clang-format on

  constexpr double tolerance = 1.0e-12;
  EXPECT_TRUE(rotation.matrix().isApprox(expected, tolerance));
}

TEST(So3ExponentialTest, AngleAboveSupportedMaximumIsRejected)
{
  const rigid_body_kinematics::NumericalPolicy policy;

  const double angle_above_maximum = std::nextafter(
    policy.maximum_exponential_angle, std::numeric_limits<double>::infinity());

  const Eigen::Vector3d rotation_vector(angle_above_maximum, 0.0, 0.0);

  try {
    (void)rigid_body_kinematics::Rotation3::from_rotation_vector(
      rotation_vector, policy);

    FAIL() << "Expected an angle above the maximum to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(So3ExponentialTest, NonFiniteRotationVectorIsRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const Eigen::Vector3d rotation_vector(nan, 0.0, 0.0);

  try {
    (void)rigid_body_kinematics::Rotation3::from_rotation_vector(
      rotation_vector);

    FAIL() << "Expected a non-finite rotation vector to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(So3ExponentialTest, ZeroSeriesThresholdIsInvalidPolicy)
{
  rigid_body_kinematics::NumericalPolicy policy;
  policy.series_angle_threshold = 0.0;

  const Eigen::Vector3d rotation_vector = Eigen::Vector3d::Zero();

  try {
    (void)rigid_body_kinematics::Rotation3::from_rotation_vector(
      rotation_vector, policy);

    FAIL() << "Expected a zero series threshold to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}

TEST(So3ExponentialTest, SeriesThresholdBoundaryMatchesAnalyticRotation)
{
  const rigid_body_kinematics::NumericalPolicy policy;
  const double threshold = policy.series_angle_threshold;

  const std::array<double, 3> angles{
    std::nextafter(threshold, 0.0), threshold,
    std::nextafter(threshold, std::numeric_limits<double>::infinity())};

  constexpr double tolerance = 1.0e-15;

  for (const double angle : angles) {
    SCOPED_TRACE(angle);
    const Eigen::Vector3d rotation_vector(0.0, 0.0, angle);

    const rigid_body_kinematics::Rotation3 rotation =
      rigid_body_kinematics::Rotation3::from_rotation_vector(
        rotation_vector, policy);

    Eigen::Matrix3d expected;

    // clang-format off
    expected <<
      std::cos(angle), -std::sin(angle), 0.0,
      std::sin(angle),  std::cos(angle), 0.0,
      0.0,              0.0,             1.0;
    // clang-format on

    EXPECT_TRUE(rotation.matrix().isApprox(expected, tolerance));
  }
}

TEST(
  So3ExponentialTest,
  ArbitraryAxisProducesProperRotationAndNegativeVectorInverse)
{
  // All three components are nonzero and measured in radians
  const Eigen::Vector3d rotation_vector(0.3, -0.4, 0.2);

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(rotation_vector);

  const rigid_body_kinematics::Rotation3 negative_rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(-rotation_vector);

  const Eigen::Matrix3d matrix = rotation.matrix();
  const Eigen::Matrix3d negative_matrix = negative_rotation.matrix();

  const Eigen::Matrix3d orthogonality_product = matrix.transpose() * matrix;

  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(
    orthogonality_product.isApprox(Eigen::Matrix3d::Identity(), tolerance));

  EXPECT_NEAR(matrix.determinant(), 1.0, tolerance);

  EXPECT_TRUE(negative_matrix.isApprox(matrix.transpose(), tolerance));
}
