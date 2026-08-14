#include <gtest/gtest.h>

#include <Eigen/Core>
#include <limits>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/lie_algebra.hpp"

TEST(LieAlgebraTest, HatSo3ProducesCrossProductMatrix)
{
  const Eigen::Vector3d vector(1.0, -2.0, 3.0);

  const Eigen::Matrix3d actual = rigid_body_kinematics::hat_so3(vector);
  // clang-format off
  Eigen::Matrix3d expected;
  expected <<
     0.0, -3.0, -2.0,
     3.0,  0.0, -1.0,
     2.0,  1.0,  0.0;
  // clang-format on

  constexpr double tolerance = 1e-12;

  EXPECT_LE((actual - expected).cwiseAbs().maxCoeff(), tolerance);

  // Independently check [x]_cross y = x cross y.
  const Eigen::Vector3d operand(4.0, 5.0, -1.0);
  const Eigen::Vector3d expected_cross_product(-13.0, 13.0, 13.0);

  const Eigen::Vector3d actual_cross_product = actual * operand;

  EXPECT_LE(
    (actual_cross_product - expected_cross_product).cwiseAbs().maxCoeff(),
    tolerance);
}

TEST(LieAlgebraTest, HatSo3ProducesSkewSymmetricMatrix)
{
  const Eigen::Vector3d vector(0.1, -0.2, 0.3);

  const Eigen::Matrix3d skew_matrix = rigid_body_kinematics::hat_so3(vector);

  const Eigen::Matrix3d skew_symmetry_error =
    skew_matrix + skew_matrix.transpose();

  EXPECT_DOUBLE_EQ(skew_symmetry_error.cwiseAbs().maxCoeff(), 0.0);
}

TEST(LieAlgebraTest, HatSo3OfZeroIsExactlyZero)
{
  const Eigen::Matrix3d actual =
    rigid_body_kinematics::hat_so3(Eigen::Vector3d::Zero());

  EXPECT_DOUBLE_EQ(actual.cwiseAbs().maxCoeff(), 0.0);
}

TEST(LieAlgebraTest, HatSo3RejectsNaNAsNonFinite)
{
  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
  vector.x() = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::hat_so3(vector);
    FAIL() << "Expected the non-finite input to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(LieAlgebraTest, VeeSo3RecoversHatInput)
{
  // vee([a]_x) = a
  const Eigen::Vector3d expected(1.0, -2.0, 3.0);

  const Eigen::Matrix3d skew_matrix = rigid_body_kinematics::hat_so3(expected);
  const Eigen::Vector3d actual = rigid_body_kinematics::vee_so3(skew_matrix);

  EXPECT_DOUBLE_EQ(actual.x(), expected.x());
  EXPECT_DOUBLE_EQ(actual.y(), expected.y());
  EXPECT_DOUBLE_EQ(actual.z(), expected.z());
}

TEST(LieAlgebraTest, VeeSo3AcceptsSmallSymmetricContamination)
{
  const Eigen::Vector3d expected(1.0, -2.0, 3.0);

  Eigen::Matrix3d matrix = rigid_body_kinematics::hat_so3(expected);

  // Add equal contamination to transposed entries.
  // This changes only the symmetric part.
  constexpr double contamination = 5.0e-13;
  matrix(0, 1) += contamination;
  matrix(1, 0) += contamination;

  rigid_body_kinematics::NumericalPolicy policy;
  policy.tangent_tolerance = 1.0e-12;

  const Eigen::Vector3d actual = rigid_body_kinematics::vee_so3(matrix, policy);

  constexpr double recovery_tolerance = 1.0e-12;

  EXPECT_LE((actual - expected).cwiseAbs().maxCoeff(), recovery_tolerance);
}

TEST(LieAlgebraTest, VeeSo3RejectsNonSkewMatrix)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();

  // A skew-symmetric matrix must have a zero diagonal.
  matrix(0, 0) = 2.0e-12;

  rigid_body_kinematics::NumericalPolicy policy;
  policy.tangent_tolerance = 1.0e-12;

  try {
    (void)rigid_body_kinematics::vee_so3(matrix, policy);
    FAIL() << "Expected the non-skew matrix to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_tangent);
  }
}

TEST(LieAlgebraTest, VeeSo3RejectsNegativeTangentTolerance)
{
  rigid_body_kinematics::NumericalPolicy policy;
  policy.tangent_tolerance = -1.0;

  const Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();

  try {
    (void)rigid_body_kinematics::vee_so3(matrix, policy);
    FAIL() << "Expected the negative tolerance to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}

TEST(LieAlgebraTest, VeeSo3RejectsNaNAsNonFinite)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix(1, 2) = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::vee_so3(matrix);
    FAIL() << "Expected the non-finite matrix to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(LieAlgebraTest, HatSe3UsesLinearFirstComponentOrder)
{
  rigid_body_kinematics::Vector6LinearFirst coordinates;
  // clang-format off
  coordinates <<
    1.0, -2.0, 3.0,
    0.1, -0.2, 0.3;
  // clang-format on

  const Eigen::Matrix4d actual = rigid_body_kinematics::hat_se3(coordinates);

  // clang-format off
  Eigen::Matrix3d expected_angular_block;
  expected_angular_block <<
    0.0, -0.3, -0.2,
    0.3,  0.0, -0.1,
    0.2,  0.1,  0.0;
  // clang-format on

  const Eigen::Vector3d expected_linear_block(1.0, -2.0, 3.0);

  const Eigen::Matrix3d actual_angular_block = actual.block<3, 3>(0, 0);

  const Eigen::Vector3d actual_linear_block = actual.block<3, 1>(0, 3);

  constexpr double angular_tolerance = 1.0e-12;
  constexpr double linear_tolerance = 1.0e-12;

  EXPECT_LE(
    (actual_angular_block - expected_angular_block).cwiseAbs().maxCoeff(),
    angular_tolerance);

  EXPECT_LE(
    (actual_linear_block - expected_linear_block).cwiseAbs().maxCoeff(),
    linear_tolerance);

  EXPECT_DOUBLE_EQ(actual(3, 0), 0.0);
  EXPECT_DOUBLE_EQ(actual(3, 1), 0.0);
  EXPECT_DOUBLE_EQ(actual(3, 2), 0.0);
  EXPECT_DOUBLE_EQ(actual(3, 3), 0.0);
}

TEST(LieAlgebraTest, HatSe3OfZeroIsExactlyZero)
{
  const rigid_body_kinematics::Vector6LinearFirst coordinates =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  const Eigen::Matrix4d actual = rigid_body_kinematics::hat_se3(coordinates);

  EXPECT_DOUBLE_EQ(actual.cwiseAbs().maxCoeff(), 0.0);
}

TEST(LieAlgebraTest, HatSe3RejectsInfinityAsNonFinite)
{
  rigid_body_kinematics::Vector6LinearFirst coordinates =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  coordinates(4) = std::numeric_limits<double>::infinity();

  try {
    (void)rigid_body_kinematics::hat_se3(coordinates);
    FAIL() << "Expected the non-finite input to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(LieAlgebraTest, VeeSe3RecoversLinearFirstHatInput)
{
  rigid_body_kinematics::Vector6LinearFirst expected;

  // First three entries are linear; last three are angular.
  // clang-format off
  expected <<
    1.0, -2.0, 3.0,
    0.1, -0.2, 0.3;
  // clang-format on

  const Eigen::Matrix4d tangent_matrix =
    rigid_body_kinematics::hat_se3(expected);

  const rigid_body_kinematics::Vector6LinearFirst actual =
    rigid_body_kinematics::vee_se3(tangent_matrix);

  const Eigen::Vector3d actual_linear = actual.head<3>();

  const Eigen::Vector3d expected_linear = expected.head<3>();

  const Eigen::Vector3d actual_angular = actual.tail<3>();

  const Eigen::Vector3d expected_angular = expected.tail<3>();

  constexpr double linear_tolerance = 1.0e-12;
  constexpr double angular_tolerance = 1.0e-12;

  EXPECT_LE(
    (actual_linear - expected_linear).cwiseAbs().maxCoeff(), linear_tolerance);

  EXPECT_LE(
    (actual_angular - expected_angular).cwiseAbs().maxCoeff(),
    angular_tolerance);
}

TEST(LieAlgebraTest, VeeSe3RejectsNonzeroBottomRow)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();

  matrix(3, 2) = 2.0e-12;

  rigid_body_kinematics::NumericalPolicy policy;
  policy.tangent_tolerance = 1.0e-12;

  try {
    (void)rigid_body_kinematics::vee_se3(matrix, policy);
    FAIL() << "Expected the nonzero bottom row to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_tangent);
  }
}

TEST(LieAlgebraTest, VeeSe3RejectsNonSkewAngularBlock)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();

  // The bottom row is valid, but a skew matrix has zero diagonal.
  matrix(1, 1) = 2.0e-12;

  rigid_body_kinematics::NumericalPolicy policy;
  policy.tangent_tolerance = 1.0e-12;

  try {
    (void)rigid_body_kinematics::vee_se3(matrix, policy);
    FAIL() << "Expected the non-skew angular block to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_tangent);
  }
}

TEST(LieAlgebraTest, VeeSe3RejectsNaNAsNonFinite)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();

  // Put NaN in the linear top-right block.
  matrix(1, 3) = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::vee_se3(matrix);
    FAIL() << "Expected the non-finite tangent matrix to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}
