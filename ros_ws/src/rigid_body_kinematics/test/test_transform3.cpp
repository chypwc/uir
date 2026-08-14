#include <gtest/gtest.h>

#include <Eigen/Core>
#include <limits>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/transform3.hpp"

TEST(Transform3Test, IdentityLeavesPointUnchanged)
{
  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::identity();

  // Point coordinates expressed in frame {b}, measured in metres
  const Eigen::Vector3d point_b(1.0, -2.0, 3.5);

  const Eigen::Vector3d point_a = transform.transform_point(point_b);

  EXPECT_DOUBLE_EQ(point_a.x(), 1.0);
  EXPECT_DOUBLE_EQ(point_a.y(), -2.0);
  EXPECT_DOUBLE_EQ(point_a.z(), 3.5);
}

TEST(Transform3Test, RotationAndTranslationTransformPoint)
{
  // T_ab is the pose of frame {b} relative to frame {a}.
  // R_ab is a +90 degree rotation about the shared positive z-axis.
  // The translation p_ab_a is [1, 2, 3]^T metres.
  Eigen::Matrix4d matrix_ab;
  // clang-format off
  matrix_ab <<
    0.0, -1.0, 0.0, 1.0,
    1.0,  0.0, 0.0, 2.0,
    0.0,  0.0, 1.0, 3.0,
    0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix_ab);

  // Coordinates of point P expressed in frame {b},
  const Eigen::Vector3d point_b(1.0, 0.0, 0.0);

  const Eigen::Vector3d point_a = transform_ab.transform_point(point_b);

  constexpr double tolerance = 1.0e-12;

  EXPECT_NEAR(point_a.x(), 1.0, tolerance);
  EXPECT_NEAR(point_a.y(), 3.0, tolerance);
  EXPECT_NEAR(point_a.z(), 3.0, tolerance);
}

TEST(Transform3Test, RejectMalformedHomogeneousBottomRow)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

  // A rigid homogeneous transform requires [0, 0, 0, 1]
  matrix(3, 0) = 1.0e-6;

  try {
    (void)rigid_body_kinematics::Transform3::from_matrix(matrix);
    FAIL() << "Expected the malformed homogeneous row to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_transform);
  }
}

TEST(Transform3Test, RejectReflectionInRotationBlock)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

  // The bottom row is valid, but the rotation block has determinant -1
  matrix(2, 2) = -1.0;

  try {
    (void)rigid_body_kinematics::Transform3::from_matrix(matrix);
    FAIL() << "Expected the reflection to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_rotation);
  }
}

TEST(Transform3Test, RejectNaNTransformEntryAsNonFinite)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 3) = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::Transform3::from_matrix(matrix);
    FAIL() << "Expected a transform containing NaN to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(Transform3Test, RejectInfinitePointAsNonFinite)
{
  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::identity();

  const Eigen::Vector3d point_b(
    std::numeric_limits<double>::infinity(), 0.0, 0.0);
  try {
    (void)transform.transform_point(point_b);
    FAIL() << "Expected the infinite point to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(Transform3Test, ReportsUnsupportedMagnitudeWhenTranslationOverflows)
{
  const double largest_finite = std::numeric_limits<double>::max();

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

  // Translation from frame {b} to frame {a}
  matrix(0, 3) = largest_finite;

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  // Finite point coordinates expressed in frame {b}
  const Eigen::Vector3d point_b(largest_finite, 0.0, 0.0);

  try {
    (void)transform.transform_point(point_b);
    FAIL() << "Expected the transformed point to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(Transform3Test, CompositionRotatesSecondTranslationBeforeAddition)
{
  // T_ab: frame {b} relative to frame {a}.
  // Translation is expressed in {a} and measured in metres.
  // clang-format off
  Eigen::Matrix4d matrix_ab;
  matrix_ab <<
    0.0, -1.0, 0.0, 1.0,
    1.0,  0.0, 0.0, 2.0,
    0.0,  0.0, 1.0, 3.0,
    0.0,  0.0, 0.0, 1.0;

  // T_bc: frame {c} relative to frame {b}.
  Eigen::Matrix4d matrix_bc;
  matrix_bc <<
    1.0, 0.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix4d expected_ac;
  expected_ac <<
    0.0, -1.0, 0.0, 1.0,
    1.0,  0.0, 0.0, 3.0,
    0.0,  0.0, 1.0, 3.0,
    0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix_ab);

  const rigid_body_kinematics::Transform3 transform_bc =
    rigid_body_kinematics::Transform3::from_matrix(matrix_bc);

  const Eigen::Matrix4d actual_ac = transform_ab.compose(transform_bc).matrix();

  const Eigen::Matrix3d actual_rotation_ac = actual_ac.block<3, 3>(0, 0);
  const Eigen::Vector3d actual_translation_ac = actual_ac.block<3, 1>(0, 3);

  const Eigen::Matrix3d expected_rotation_ac = expected_ac.block<3, 3>(0, 0);
  const Eigen::Vector3d expected_translation_ac = expected_ac.block<3, 1>(0, 3);

  constexpr double rotation_tolerance = 1e-12;
  constexpr double translation_tolerance = 1e-12;

  EXPECT_LE(
    (actual_rotation_ac - expected_rotation_ac).cwiseAbs().maxCoeff(),
    rotation_tolerance);

  EXPECT_LE(
    (actual_translation_ac - expected_translation_ac).cwiseAbs().maxCoeff(),
    translation_tolerance);

  EXPECT_DOUBLE_EQ(actual_ac(3, 0), 0.0);
  EXPECT_DOUBLE_EQ(actual_ac(3, 1), 0.0);
  EXPECT_DOUBLE_EQ(actual_ac(3, 2), 0.0);
  EXPECT_DOUBLE_EQ(actual_ac(3, 3), 1.0);
}

TEST(Transform3Test, InverseMatchesAnalyticResultAndComposesToIdentity)
{
  // T_ab has a +90-degree rotation about +z and translation
  // [1, 2, 3]^T metres, expressed in frame {a}.
  // clang-format off
  Eigen::Matrix4d matrix_ab;
  matrix_ab <<
    0.0, -1.0, 0.0, 1.0,
    1.0,  0.0, 0.0, 2.0,
    0.0,  0.0, 1.0, 3.0,
    0.0,  0.0, 0.0, 1.0;

  Eigen::Matrix4d expected_ba;
  expected_ba <<
     0.0, 1.0, 0.0, -2.0,
    -1.0, 0.0, 0.0,  1.0,
     0.0, 0.0, 1.0, -3.0,
     0.0, 0.0, 0.0,  1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix_ab);

  const rigid_body_kinematics::Transform3 transform_ba = transform_ab.inverse();

  const Eigen::Matrix4d matrix_ba = transform_ba.matrix();

  const Eigen::Matrix3d actual_rotation_ba = matrix_ba.block<3, 3>(0, 0);
  const Eigen::Vector3d actual_translation_ba = matrix_ba.block<3, 1>(0, 3);

  const Eigen::Matrix3d expected_rotation_ba = expected_ba.block<3, 3>(0, 0);
  const Eigen::Vector3d expected_translation_ba = expected_ba.block<3, 1>(0, 3);

  const Eigen::Matrix4d right_identity =
    transform_ab.compose(transform_ba).matrix();
  const Eigen::Matrix4d left_identity =
    transform_ba.compose(transform_ab).matrix();

  const Eigen::Matrix3d right_rotation = right_identity.block<3, 3>(0, 0);
  const Eigen::Vector3d right_translation = right_identity.block<3, 1>(0, 3);
  const Eigen::Matrix3d left_rotation = left_identity.block<3, 3>(0, 0);
  const Eigen::Vector3d left_translation = left_identity.block<3, 1>(0, 3);

  constexpr double rotation_tolerance = 1e-12;
  constexpr double translation_tolerance = 1e-12;

  EXPECT_LE(
    (actual_rotation_ba - expected_rotation_ba).cwiseAbs().maxCoeff(),
    rotation_tolerance);

  EXPECT_LE(
    (actual_translation_ba - expected_translation_ba).cwiseAbs().maxCoeff(),
    translation_tolerance);

  EXPECT_LE(
    (right_rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),
    rotation_tolerance);

  EXPECT_LE(right_translation.cwiseAbs().maxCoeff(), translation_tolerance);

  EXPECT_LE(
    (left_rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),
    rotation_tolerance);

  EXPECT_LE(left_translation.cwiseAbs().maxCoeff(), translation_tolerance);
}

TEST(Transform3Test, ReportsUnsupportedMagnitudeWhenCompositionOverflows)
{
  const double largest_finite = std::numeric_limits<double>::max();

  Eigen::Matrix4d matrix_ab = Eigen::Matrix4d::Identity();
  matrix_ab(0, 3) = largest_finite;

  Eigen::Matrix4d matrix_bc = Eigen::Matrix4d::Identity();
  matrix_bc(0, 3) = largest_finite;

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix_ab);

  const rigid_body_kinematics::Transform3 transform_bc =
    rigid_body_kinematics::Transform3::from_matrix(matrix_bc);

  try {
    (void)transform_ab.compose(transform_bc);
    FAIL() << "Expected the composed translation to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}
