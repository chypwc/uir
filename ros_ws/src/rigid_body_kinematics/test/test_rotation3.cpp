#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/rotation3.hpp"

TEST(Rotation3Test, IdentityLeavesFreeVectorUnchanged)
{
  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::identity();

  const Eigen::Vector3d vector_b(1.0, -2.0, 3.5);

  const Eigen::Vector3d vector_a = rotation.rotate_vector(vector_b);

  EXPECT_DOUBLE_EQ(vector_a.x(), 1.0);
  EXPECT_DOUBLE_EQ(vector_a.y(), -2.0);
  EXPECT_DOUBLE_EQ(vector_a.z(), 3.5);
}

TEST(Rotation3Test, ValidatedQuarterTurnRotatesFreeVector)
{
  // R_ab maps components from frame {b} into frame {a}.
  // It represents +90 degrees about the shared positive z-axis.
  Eigen::Matrix3d matrix_ab;
  matrix_ab << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  const rigid_body_kinematics::Rotation3 rotation_ab =
    rigid_body_kinematics::Rotation3::from_matrix(matrix_ab);

  // Components of the same free vector, measured in metres
  const Eigen::Vector3d vector_b(1.0, 0.0, 0.0);

  const Eigen::Vector3d vector_a = rotation_ab.rotate_vector(vector_b);

  constexpr double tolerance = 1.0e-12;

  EXPECT_NEAR(vector_a.x(), 0.0, tolerance);
  EXPECT_NEAR(vector_a.y(), 1.0, tolerance);
  EXPECT_NEAR(vector_a.z(), 0.0, tolerance);
}

TEST(Rotation3Test, RejectReflectionAsInvalidRotation)
{
  // This dimensionless matrix is orthogonal, but det(R) = -1.
  // It is a reflection, not a member of SO(3).
  Eigen::Matrix3d reflection;
  reflection << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;

  try {
    (void)rigid_body_kinematics::Rotation3::from_matrix(reflection);
    FAIL() << "Expected the reflection to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_rotation);
  }
}

TEST(Rotation3Test, RejectNonOrthogonalMatrixAsInvalidRotation)
{
  // This dimensionless shear has det(R) = 1, but R^T R != I.
  Eigen::Matrix3d shear;
  shear << 1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  try {
    (void)rigid_body_kinematics::Rotation3::from_matrix(shear);
    FAIL() << "Expected the non-orthogonal matrix to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_rotation);
  }
}

TEST(Rotation3Test, RejectNaNEntryAsNonFinite)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  matrix(0, 0) = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::Rotation3::from_matrix(matrix);
    FAIL() << "Expected a matrix containing NaN to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(Rotation3Test, RejectsInvalidPolicyBeforeInspectingMatrix)
{
  rigid_body_kinematics::NumericalPolicy policy;
  policy.orthogonality_tolerance = -1.0;

  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  matrix(0, 0) = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::Rotation3::from_matrix(matrix, policy);
    FAIL() << "Expected the invalid policy to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}

TEST(Rotation3Test, ReportsUnsupportedMagnitudeWhenValidationOverflows)
{
  // Every input entry is finite, but max_double squared is infinite.
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  matrix(0, 0) = std::numeric_limits<double>::max();

  try {
    (void)rigid_body_kinematics::Rotation3::from_matrix(matrix);
    FAIL() << "Expected numerical overflow to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(Rotation3Test, RejectsInfiniteFreeVectorAsNonFinite)
{
  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::identity();

  const Eigen::Vector3d vector_b(
    std::numeric_limits<double>::infinity(), 0.0, 0.0);

  try {
    (void)rotation.rotate_vector(vector_b);
    FAIL() << "Expected the infinite free vector to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(Rotation3Test, ReportsUnsupportedMagnitudeWhenVectorRotationOverflows)
{
  const double coefficient = std::sqrt(0.5);

  // 45 degree rotation about +z-axis
  Eigen::Matrix3d matrix_ab;
  matrix_ab << coefficient, -coefficient, 0.0, coefficient, coefficient, 0.0,
    0.0, 0.0, 1.0;

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_matrix(matrix_ab);

  const double largest_finite = std::numeric_limits<double>::max();

  const Eigen::Vector3d vector_b(largest_finite, largest_finite, 0.0);

  try {
    (void)rotation.rotate_vector(vector_b);
    FAIL() << "Expected the rotated result to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(Rotation3Test, CompositionIsRightmostFirstAndNoncommuting)
{
  // clang-format off
  Eigen::Matrix3d matrix_z;
  matrix_z <<
    0.0, -1.0, 0.0,
    1.0,  0.0, 0.0,
    0.0,  0.0, 1.0;

  Eigen::Matrix3d matrix_x;
  matrix_x <<
    1.0, 0.0,  0.0,
    0.0, 0.0, -1.0,
    0.0, 1.0,  0.0;
  // clang-format on

  const rigid_body_kinematics::Rotation3 rotation_z =
    rigid_body_kinematics::Rotation3::from_matrix(matrix_z);

  const rigid_body_kinematics::Rotation3 rotation_x =
    rigid_body_kinematics::Rotation3::from_matrix(matrix_x);

  // clang-format off
  Eigen::Matrix3d expected_zx;
  expected_zx <<
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0;

  Eigen::Matrix3d expected_xz;
  expected_xz <<
    0.0, -1.0,  0.0,
    0.0,  0.0, -1.0,
    1.0,  0.0,  0.0;
  // clang-format on

  const Eigen::Matrix3d actual_zx = rotation_z.compose(rotation_x).matrix();
  const Eigen::Matrix3d actual_xz = rotation_x.compose(rotation_z).matrix();

  constexpr double tolerance = 1e-12;
  EXPECT_TRUE(actual_zx.isApprox(expected_zx, tolerance));
  EXPECT_TRUE(actual_xz.isApprox(expected_xz, tolerance));
  EXPECT_FALSE(actual_zx.isApprox(actual_xz, tolerance));
}

TEST(Rotation3Test, InverseComposesToIdentityOnBothSides)
{
  const double coefficient = std::sqrt(0.5);

  // A 45-degree rotation about the positive z-axis
  // clang-format off
  Eigen::Matrix3d matrix_ab;
  matrix_ab <<
    coefficient, -coefficient, 0.0,
    coefficient,  coefficient, 0.0,
    0.0,          0.0,         1.0;
  // clang-format on
  const rigid_body_kinematics::Rotation3 rotation_ab =
    rigid_body_kinematics::Rotation3::from_matrix(matrix_ab);

  const rigid_body_kinematics::Rotation3 rotation_ba = rotation_ab.inverse();

  const Eigen::Matrix3d right_identity =
    rotation_ab.compose(rotation_ba).matrix();
  const Eigen::Matrix3d left_identity =
    rotation_ba.compose(rotation_ab).matrix();

  constexpr double tolerance = 1e-12;
  const Eigen::Matrix3d expected_identity = Eigen::Matrix3d::Identity();

  EXPECT_TRUE(right_identity.isApprox(expected_identity, tolerance));
  EXPECT_TRUE(left_identity.isApprox(expected_identity, tolerance));
}
