#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <numbers>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/transform3.hpp"

TEST(Se3ExponentialTest, PureTranslationPreservesLinearCoordinatesExactly)
{
  rigid_body_kinematics::Vector6LinearFirst coordinates;

  // eta = [rho; phi], with rho in metres and phi in radians.
  coordinates << 0.4, -0.2, 0.1, 0.0, 0.0, 0.0;

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_exponential_coordinates(
      coordinates);

  Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
  expected.block<3, 1>(0, 3) = coordinates.head<3>();

  const Eigen::Matrix4d actual = transform.matrix();

  EXPECT_TRUE((actual.array() == expected.array()).all());
}

TEST(Se3ExponentialTest, FinitePitchScrewMatchesAnalyticTransform)
{
  const double pi = std::numbers::pi;
  const double half_pi = 0.5 * pi;

  rigid_body_kinematics::Vector6LinearFirst coordinates;

  // rho = s_v * theta and phi = s_omega * theta.
  // clang-format off
  coordinates <<
    0.0, -pi, pi / 40.0,  // rho
    0.0, 0.0, half_pi;    // phi, theta = pi / 2.
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_exponential_coordinates(
      coordinates);

  // At theta = pi/2, the xy block of J_z is
  // [ 2/pi, -2/pi ]
  // [ 2/pi,  2/pi ].
  //
  // Therefore J_z [0, -pi, pi/40]^T = [2, -2, pi/40]^T.
  Eigen::Matrix4d expected;

  // clang-format off
  expected <<
    0.0, -1.0, 0.0, 2.0,
    1.0, 0.0, 0.0, -2.0,
    0.0, 0.0, 1.0, pi / 40.0,
    0.0, 0.0, 0.0, 1.0;
  // clang-format on

  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(transform.matrix().isApprox(expected, tolerance));
}

TEST(Se3ExponentialTest, SmallNonzeroRotationUsesStableLeftJacobianSeries)
{
  constexpr double theta = 1.0e-8;  // radians

  rigid_body_kinematics::Vector6LinearFirst coordinates;

  // clang-format off
  coordinates <<
    0.3, -0.2, 0.1,    // rho in metres
    0.0,  0.0, theta;  // phi in radians
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_exponential_coordinates(
      coordinates);

  const Eigen::Vector3d linear_coordinates = coordinates.head<3>();

  // Independent stable coefficients for the analytic z-axis Jacobian:
  //
  // a = sin(theta) / theta,
  // d = (1 - cos(theta)) / theta
  //   = 2 sin^2(theta / 2) / theta.
  //
  // The second form for d avoids subtracting two nearly equal numbers.
  const double sine_ratio = std::sin(theta) / theta;
  const double one_minus_cosine_ratio =
    2.0 * std::sin(0.5 * theta) * std::sin(0.5 * theta) / theta;

  // J(phi) * rho.
  const Eigen::Vector3d expected_translation(
    sine_ratio * linear_coordinates(0) -
      one_minus_cosine_ratio * linear_coordinates(1),
    one_minus_cosine_ratio * linear_coordinates(0) +
      sine_ratio * linear_coordinates(1),
    linear_coordinates(2));

  Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();

  // Independent analytic rotation about the positive z-axis.
  // clang-format off
  expected.block<3, 3>(0, 0) <<
    std::cos(theta), -std::sin(theta), 0.0,
    std::sin(theta),  std::cos(theta), 0.0,
    0.0,              0.0,             1.0;
  // clang-format on

  expected.block<3, 1>(0, 3) = expected_translation;

  constexpr double tolerance = 1.0e-15;
  EXPECT_TRUE(transform.matrix().isApprox(expected, tolerance));
}

TEST(Se3ExponentialTest, RejectsNonFiniteLinearCoordinate)
{
  rigid_body_kinematics::Vector6LinearFirst coordinates =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  coordinates(0) = std::numeric_limits<double>::infinity();

  try {
    (void)rigid_body_kinematics::Transform3::from_exponential_coordinates(
      coordinates);

    FAIL() << "Expected a non-finite linear coordinate to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(Se3ExponentialTest, ReportsUnsupportedMagnitudeWhenTranslationOverflows)
{
  const double largest_finite = std::numeric_limits<double>::max();
  const double half_pi = 0.5 * std::numbers::pi;

  rigid_body_kinematics::Vector6LinearFirst coordinates;

  // For phi = [0, 0, pi/2], both entries in the second row of the
  // xy Jacobian block are 2/pi. Multiplying by [max, max] therefore
  // adds two large positive finite terms and overflows.
  // clang-format off
  coordinates <<
    largest_finite, largest_finite, 0.0,
    0.0,            0.0,            half_pi;
  // clang-format on

  try {
    (void)rigid_body_kinematics::Transform3::from_exponential_coordinates(
      coordinates);

    FAIL() << "Expected the exponential translation to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}
