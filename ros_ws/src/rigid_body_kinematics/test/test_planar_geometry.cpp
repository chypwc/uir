#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/planar_geometry.hpp"

TEST(PlanarGeometryTest, AcceptanceCaseEmbedsQuarterTurnPose)
{
  const rigid_body_kinematics::PlanarPose planar_pose{
    2.0, -1.0, std::numbers::pi / 2.0};

  const Eigen::Matrix4d actual =
    rigid_body_kinematics::embed_planar_pose(planar_pose).matrix();

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
    0.0, -1.0, 0.0,  2.0,
    1.0,  0.0, 0.0, -1.0,
    0.0,  0.0, 1.0,  0.0,
    0.0,  0.0, 0.0,  1.0;
  // clang-format on

  constexpr double tolerance = 1.0e-12;
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      EXPECT_NEAR(actual(row, column), expected(row, column), tolerance);
    }
  }
}

TEST(PlanarGeometryTest, ZeroPoseProducesExactIdentity)
{
  const rigid_body_kinematics::PlanarPose planar_pose{0.0, 0.0, 0.0};

  const Eigen::Matrix4d actual =
    rigid_body_kinematics::embed_planar_pose(planar_pose).matrix();

  const Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();

  EXPECT_EQ(actual, expected);
}

TEST(PlanarGeometryTest, RejectsNaNPoseComponentAsNonFinite)
{
  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();

  const rigid_body_kinematics::PlanarPose planar_pose{quiet_nan, 0.0, 0.0};

  try {
    (void)rigid_body_kinematics::embed_planar_pose(planar_pose);
    FAIL() << "Expected a non-finite planar pose to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(PlanarGeometryTest, RejectsInfiniteYawAsNonFinite)
{
  const double infinity = std::numeric_limits<double>::infinity();

  const rigid_body_kinematics::PlanarPose planar_pose{0.0, 0.0, infinity};

  try {
    (void)rigid_body_kinematics::embed_planar_pose(planar_pose);
    FAIL() << "Expected infinite planar yaw to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(PlanarGeometryTest, RejectsYawAboveSupportedMaximum)
{
  const rigid_body_kinematics::NumericalPolicy policy{};

  const double yaw_above_maximum = std::nextafter(
    policy.maximum_exponential_angle, std::numeric_limits<double>::infinity());

  const rigid_body_kinematics::PlanarPose planar_pose{
    0.0, 0.0, yaw_above_maximum};

  try {
    (void)rigid_body_kinematics::embed_planar_pose(planar_pose, policy);
    FAIL() << "Expected yaw above the supported maximum to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(PlanarGeometryTest, AcceptsYawAtSupportedMaximum)
{
  const rigid_body_kinematics::NumericalPolicy policy{};

  const rigid_body_kinematics::PlanarPose planar_pose{
    0.0, 0.0, policy.maximum_exponential_angle};

  EXPECT_NO_THROW(
    { (void)rigid_body_kinematics::embed_planar_pose(planar_pose, policy); });
}

// verify that policy validation occurs before pose validation.
TEST(PlanarGeometryTest, RejectsInvalidPolicyBeforeInspectingPose)
{
  rigid_body_kinematics::NumericalPolicy policy{};
  policy.maximum_exponential_angle = 0.0;

  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();

  const rigid_body_kinematics::PlanarPose planar_pose{quiet_nan, 0.0, 0.0};

  try {
    (void)rigid_body_kinematics::embed_planar_pose(planar_pose, policy);
    FAIL() << "Expected the invalid yaw policy to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}

TEST(PlanarGeometryTest, AcceptanceCaseExtractsQuarterTurnPose)
{
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    0.0, -1.0, 0.0,  2.0,
    1.0,  0.0, 0.0, -1.0,
    0.0,  0.0, 1.0,  0.0,
    0.0,  0.0, 0.0,  1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::PlanarPose actual =
    rigid_body_kinematics::extract_planar_pose(transform);

  constexpr double position_tolerance = 1.0e-12;
  constexpr double angle_tolerance = 1.0e-12;

  EXPECT_NEAR(actual.x_metres, 2.0, position_tolerance);
  EXPECT_NEAR(actual.y_metres, -1.0, position_tolerance);
  EXPECT_NEAR(actual.yaw_radians, 0.5 * std::numbers::pi, angle_tolerance);
}

TEST(PlanarGeometryTest, RejectsVerticalTranslationAsNonPlanar)
{
  // |p_z|=10^{-6} > 10^{-12} planar_position_tolerance.
  constexpr double vertical_offset_metres = 1.0e-6;

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(2, 3) = vertical_offset_metres;

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  try {
    (void)rigid_body_kinematics::extract_planar_pose(transform);
    FAIL() << "Expected vertical translation to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_planar);
  }
}

TEST(PlanarGeometryTest, RejectsRollAsNonPlanar)
{
  // "roll" means a rotation about the x-axis.
  // Planar motion permits only yaw: rotation about the z-axis.
  constexpr double roll_radians = 1.0e-6;

  const double cosine = std::cos(roll_radians);
  const double sine = std::sin(roll_radians);

  // Planar orientation does not preserve the positive z-axis.
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    1.0, 0.0,     0.0,    0.0,
    0.0, cosine,  -sine,   0.0,
    0.0, sine,     cosine, 0.0,
    0.0, 0.0,      0.0,    1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  try {
    (void)rigid_body_kinematics::extract_planar_pose(transform);
    FAIL() << "Expected roll to be rejected as non-planar.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_planar);
  }
}

// test the promised yaw range (-π, π]
TEST(PlanarGeometryTest, CanonicalizesNegativePiYawToPositivePi)
{
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    -1.0,  0.0, 0.0, 0.0,
    -0.0, -1.0, 0.0, 0.0,  // atan2(-0.0, -1.0) for -pi
     0.0,  0.0, 1.0, 0.0,
     0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::PlanarPose actual =
    rigid_body_kinematics::extract_planar_pose(transform);

  // verify the test fixture preserves the sign
  // std::signbit(-0.0)  // true: sign bit is 1
  ASSERT_TRUE(std::signbit(transform.matrix()(1, 0)));

  EXPECT_EQ(actual.x_metres, 0.0);
  EXPECT_EQ(actual.y_metres, 0.0);
  EXPECT_EQ(actual.yaw_radians, std::numbers::pi);
}

TEST(PlanarGeometryTest, RejectsNegativePlanarToleranceAsInvalidPolicy)
{
  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::identity();

  rigid_body_kinematics::NumericalPolicy policy{};
  policy.planar_orientation_tolerance = -1.0e-12;

  try {
    (void)rigid_body_kinematics::extract_planar_pose(transform, policy);
    FAIL() << "Expected a negative planar tolerance to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}

TEST(PlanarGeometryTest, QuarterTurnYawProducesNormalizedRosOrderQuaternion)
{
  constexpr double tolerance = 1.0e-12;

  const rigid_body_kinematics::QuaternionXYZW quaternion =
    rigid_body_kinematics::planar_yaw_to_quaternion(0.5 * std::numbers::pi);

  const double expected_component = std::sqrt(0.5);

  EXPECT_EQ(quaternion.x, 0.0);
  EXPECT_EQ(quaternion.y, 0.0);
  EXPECT_NEAR(quaternion.z, expected_component, tolerance);
  EXPECT_NEAR(quaternion.w, expected_component, tolerance);

  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;

  EXPECT_NEAR(norm_squared, 1.0, tolerance);
}

// Test q = -q, since sin((θ + 2π)/2) = sin(θ/2 + π) = - sin(θ/2).
TEST(PlanarGeometryTest, YawPlusFullTurnProducesSignEquivalentQuaternion)
{
  constexpr double tolerance = 1.0e-12;

  const rigid_body_kinematics::QuaternionXYZW first =
    rigid_body_kinematics::planar_yaw_to_quaternion(0.5 * std::numbers::pi);

  const rigid_body_kinematics::QuaternionXYZW second =
    rigid_body_kinematics::planar_yaw_to_quaternion(
      0.5 * std::numbers::pi + 2.0 * std::numbers::pi);

  EXPECT_NEAR(second.x, -first.x, tolerance);
  EXPECT_NEAR(second.y, -first.y, tolerance);
  EXPECT_NEAR(second.z, -first.z, tolerance);
  EXPECT_NEAR(second.w, -first.w, tolerance);

  const double second_norm_squared = second.x * second.x + second.y * second.y +
                                     second.z * second.z + second.w * second.w;

  EXPECT_NEAR(second_norm_squared, 1.0, tolerance);

  // For unit quaternions, an absolute dot product of one means
  // they differ only by sign: |<q_1, q_2>| = 1.
  const double dot_product = first.x * second.x + first.y * second.y +
                             first.z * second.z + first.w * second.w;

  EXPECT_NEAR(std::abs(dot_product), 1.0, tolerance);
}

// Invalid policy must be rejected before yaw is inspected.
TEST(PlanarGeometryTest, RejectsInvalidQuaternionPolicyBeforeInspectingYaw)
{
  // maximum_exponential_angle = 0 violates θ_max >= π.
  rigid_body_kinematics::NumericalPolicy policy{};
  policy.maximum_exponential_angle = 0.0;

  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::planar_yaw_to_quaternion(quiet_nan, policy);
    FAIL() << "Expected the invalid quaternion policy to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}
