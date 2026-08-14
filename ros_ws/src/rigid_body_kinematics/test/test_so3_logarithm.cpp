#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/rotation3.hpp"

TEST(So3LogarithmTest, IdentityReturnsExactZeroAndIdentityBranch)
{
  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::identity();

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector();

  EXPECT_TRUE(
    (result.rotation_vector.array() == Eigen::Vector3d::Zero().array()).all());

  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::identity);
}

TEST(So3LogarithmTest, QuarterTurnAboutZUsesNominalBranch)
{
  const double half_pi = 0.5 * std::acos(-1.0);

  Eigen::Matrix3d matrix;

  // Analytic active rotation of +pi/2 about the z-axis.
  // clang-format off
  matrix <<
    0.0, -1.0, 0.0,
    1.0,  0.0, 0.0,
    0.0,  0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_matrix(matrix);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector();

  const Eigen::Vector3d expected(0.0, 0.0, half_pi);
  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::nominal);
}

TEST(So3LogarithmTest, SmallNonzeroRotationUsesSmallAngleBranch)
{
  const rigid_body_kinematics::NumericalPolicy policy;
  const double angle = 0.5 * policy.series_angle_threshold;

  Eigen::Matrix3d matrix;

  // Analytic active rotation about the positive z-axis.
  // clang-format off
  matrix <<
    std::cos(angle), -std::sin(angle), 0.0,
    std::sin(angle),  std::cos(angle), 0.0,
    0.0,              0.0,             1.0;
  // clang-format on

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_matrix(matrix);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector(policy);

  const Eigen::Vector3d expected(0.0, 0.0, angle);
  constexpr double tolerance = 1.0e-14;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  EXPECT_EQ(
    result.branch, rigid_body_kinematics::RotationLogBranch::small_angle);
}

TEST(So3LogarithmTest, HalfTurnAboutXUsesDeterministicNearPiBranch)
{
  Eigen::Matrix3d matrix;

  // Exact rotation of pi radians about the x-axis.
  // clang-format off
  matrix <<
    1.0,  0.0,  0.0,
    0.0, -1.0,  0.0,
    0.0,  0.0, -1.0;
  // clang-format on

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_matrix(matrix);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector();

  const double pi = std::acos(-1.0);
  const Eigen::Vector3d expected(pi, 0.0, 0.0);
  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::near_pi);
}

TEST(So3LogarithmTest, NearPiRotationRecoversAxisSignFromSkewPart)
{
  const rigid_body_kinematics::NumericalPolicy policy;
  const double pi = std::acos(-1.0);
  const double angle = pi - 0.5 * policy.near_pi_tolerance;

  // Unit axis with a negative dominant component.
  const double inverse_norm = 1.0 / std::sqrt(14.0);
  const Eigen::Vector3d axis(
    -inverse_norm, 2.0 * inverse_norm, -3.0 * inverse_norm);

  const Eigen::Vector3d expected = angle * axis;

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(expected, policy);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector(policy);

  constexpr double tolerance = 1.0e-9;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::near_pi);
}

TEST(So3LogarithmTest, RotationBeyondPiReturnsEquivalentPrincipalVector)
{
  const double pi = std::acos(-1.0);

  // A +3pi/2 rotation about z is equivalent to -pi/2 about z.
  const Eigen::Vector3d input(0.0, 0.0, 1.5 * pi);

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(input);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector();

  const Eigen::Vector3d expected(0.0, 0.0, -0.5 * pi);
  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::nominal);
}

TEST(So3LogarithmTest, ArbitraryPrincipalVectorRoundTripsThroughLogarithm)
{
  // Rotation-vector components are in radians.
  const Eigen::Vector3d expected(0.3, -0.4, 0.2);

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_rotation_vector(expected);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector();

  const rigid_body_kinematics::Rotation3 reconstructed =
    rigid_body_kinematics::Rotation3::from_rotation_vector(
      result.rotation_vector);

  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  EXPECT_TRUE(reconstructed.matrix().isApprox(rotation.matrix(), tolerance));

  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::nominal);
}

TEST(So3LogarithmTest, HalfTurnTieUsesLowestDominantAxisIndex)
{
  Eigen::Matrix3d matrix;

  // A rotation vector satisfies phi = theta * s, where theta is the rotation
  // angle and s is the unit rotation axis. Here theta = pi, so
  //
  //   s = phi / pi = [1, 1, 0]^T / sqrt(2).
  //   phi = [1, 1, 0]^T pi  / sqrt(2)
  //
  // Its axis outer product is therefore
  //
  //   Q = s * s^T = [1/2, 1/2, 0;
  //                    1/2, 1/2, 0;
  //                    0,   0,   0].
  //
  // At theta = pi, Rodrigues' formula becomes R = 2 Q - I because
  // cos(pi) = -1 and sin(pi) = 0. This gives the matrix below.
  // clang-format off
  matrix <<
    0.0, 1.0,  0.0,
    1.0, 0.0,  0.0,
    0.0, 0.0, -1.0;
  // clang-format on

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::from_matrix(matrix);

  const rigid_body_kinematics::RotationLogResult result =
    rotation.to_principal_rotation_vector();

  // Q(0, 0) and Q(1, 1) are both 1/2. The implementation resolves this
  // tie by selecting the lowest index, x, and making that component positive.
  const double pi = std::acos(-1.0);  // theta = pi
  const double component = pi / std::sqrt(2.0);
  const Eigen::Vector3d expected(component, component, 0.0);  // phi

  constexpr double tolerance = 1.0e-12;

  EXPECT_TRUE(result.rotation_vector.isApprox(expected, tolerance));

  // theta = pi
  EXPECT_EQ(result.branch, rigid_body_kinematics::RotationLogBranch::near_pi);
}

TEST(So3LogarithmTest, OverlappingNumericalBranchesAreRejected)
{
  rigid_body_kinematics::NumericalPolicy policy;

  // These thresholds leave no nominal interval:
  // series threshold + near-pi tolerance > pi.
  policy.series_angle_threshold = 2.0;
  policy.near_pi_tolerance = 2.0;

  const rigid_body_kinematics::Rotation3 rotation =
    rigid_body_kinematics::Rotation3::identity();

  try {
    (void)rotation.to_principal_rotation_vector(policy);

    FAIL() << "Expected overlapping logarithm branches to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}
