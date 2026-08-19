#include <gtest/gtest.h>

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <limits>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/transform3.hpp"

TEST(
  Se3LogarithmTest,
  PureTranslationPreservesTranslationAndUsesPureTranslationBranch)
{
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    1.0, 0.0, 0.0, 0.4,
    0.0, 1.0, 0.0, -0.2,
    0.0, 0.0, 1.0, 0.1,
    0.0, 0.0, 0.0, 1.0;
  // clang-format on
  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::TransformLogResult actual =
    transform.to_principal_exponential_coordinates();

  rigid_body_kinematics::Vector6LinearFirst expected_coordinates;
  expected_coordinates << 0.4, -0.2, 0.1, 0.0, 0.0, 0.0;

  EXPECT_EQ(
    actual.branch, rigid_body_kinematics::TransformLogBranch::pure_translation);

  EXPECT_EQ(actual.coordinates, expected_coordinates);
}

TEST(Se3LogarithmTest, IdentityReturnsExactZeroAndIdentityBranch)
{
  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::identity();

  const rigid_body_kinematics::TransformLogResult actual =
    transform.to_principal_exponential_coordinates();

  const rigid_body_kinematics::Vector6LinearFirst expected_coordinates =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  EXPECT_EQ(actual.branch, rigid_body_kinematics::TransformLogBranch::identity);

  EXPECT_EQ(actual.coordinates, expected_coordinates);
}

TEST(Se3LogarithmTest, FinitePitchQuarterTurnRecoversNominalCoordinates)
{
  const double pi = std::acos(-1.0);
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    0.0, -1.0, 0.0, 2.0,
    1.0,  0.0, 0.0, -2.0,
    0.0,  0.0, 1.0, pi/40.0,
    0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::TransformLogResult actual =
    transform.to_principal_exponential_coordinates();

  const rigid_body_kinematics::Vector6LinearFirst expected_coordinates(
    0.0, -pi, pi / 40.0, 0.0, 0.0, 0.5 * pi);

  EXPECT_EQ(actual.branch, rigid_body_kinematics::TransformLogBranch::nominal);

  constexpr double tolerance = 1.0e-12;
  EXPECT_TRUE(actual.coordinates.isApprox(expected_coordinates, tolerance));
}

TEST(
  Se3LogarithmTest, SmallNonzeroScrewRecoversCoordinatesAndUsesSmallAngleBranch)
{
  // Construct a known small-angle screw without using the production
  // exponential, so this test independently checks the logarithm.
  constexpr double theta = 1.0e-8;
  const Eigen::Vector3d rho(0.3, -0.2, 0.1);

  const double sine_theta = std::sin(theta);
  const double cosine_theta = std::cos(theta);

  // For phi = [0, 0, theta]^T, p = J(phi) rho contains the two ratios below.
  const double sine_over_angle = sine_theta / theta;

  // Evaluate (1 - cos(theta)) / theta through the equivalent half-angle
  // expression to avoid cancellation when theta is close to zero.
  const double sine_half_angle = std::sin(0.5 * theta);
  const double one_minus_cosine_over_angle =
    2.0 * sine_half_angle * sine_half_angle / theta;

  const double translation_x =
    sine_over_angle * rho.x() - one_minus_cosine_over_angle * rho.y();

  const double translation_y =
    one_minus_cosine_over_angle * rho.x() + sine_over_angle * rho.y();

  // This matrix is the analytic finite transform (R_z(theta), J(phi) rho)
  // corresponding to the expected linear-first exponential coordinates.
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    cosine_theta, -sine_theta, 0.0, translation_x,
    sine_theta,  cosine_theta, 0.0, translation_y,
    0.0,          0.0,         1.0, rho.z(),
    0.0,          0.0,         0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::TransformLogResult actual =
    transform.to_principal_exponential_coordinates();

  const rigid_body_kinematics::Vector6LinearFirst expected_coordinates(
    rho.x(), rho.y(), rho.z(), 0.0, 0.0, theta);

  // The angle is nonzero but below the policy threshold, so both the recovered
  // coordinates and the reported numerical branch form part of the contract.
  EXPECT_EQ(
    actual.branch, rigid_body_kinematics::TransformLogBranch::small_angle);

  constexpr double tolerance = 1.0e-12;
  EXPECT_TRUE(actual.coordinates.isApprox(expected_coordinates, tolerance));
}

TEST(Se3LogarithmTest, HalfTurnScrewRecoversCoordinatesAndUsesNearPiBranch)
{
  const double pi = std::acos(-1.0);

  // Select rho and the canonical exact-pi rotation vector phi = pi * e_z.
  const Eigen::Vector3d rho(0.3, -0.2, 0.1);

  // At theta = pi about z,
  //
  // p_x = -(2 / pi) rho_y,
  // p_y =  (2 / pi) rho_x,
  // p_z = rho_z.
  //
  // Construct the transform analytically rather than using the production
  // exponential, so the logarithm is checked independently.
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    -1.0,  0.0, 0.0, 0.4 / pi,
     0.0, -1.0, 0.0, 0.6 / pi,
     0.0,  0.0, 1.0, 0.1,
     0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::TransformLogResult actual =
    transform.to_principal_exponential_coordinates();

  const rigid_body_kinematics::Vector6LinearFirst expected_coordinates(
    rho.x(), rho.y(), rho.z(), 0.0, 0.0, pi);

  // An exact half-turn lies in the near-pi region. For this coordinate-axis
  // case, the deterministic principal convention selects the +z axis.
  EXPECT_EQ(actual.branch, rigid_body_kinematics::TransformLogBranch::near_pi);

  constexpr double tolerance = 1.0e-12;
  EXPECT_TRUE(actual.coordinates.isApprox(expected_coordinates, tolerance));
}

TEST(
  Se3LogarithmTest,
  ArbitraryPrincipalCoordinatesRoundTripThroughExponentialAndLogarithm)
{
  // The rotation-vector norm is sqrt(0.5) rad, so the input is principal,
  // nonzero, and safely inside the nominal branch.
  const rigid_body_kinematics::Vector6LinearFirst expected_coordinates(
    0.7, -0.4, 1.2, 0.3, -0.4, 0.5);

  const rigid_body_kinematics::Transform3 original =
    rigid_body_kinematics::Transform3::from_exponential_coordinates(
      expected_coordinates);

  const rigid_body_kinematics::TransformLogResult logarithm =
    original.to_principal_exponential_coordinates();

  EXPECT_EQ(
    logarithm.branch, rigid_body_kinematics::TransformLogBranch::nominal);

  // Because ||phi|| < pi, Log(Exp([rho; phi])) should recover the original
  // coordinates without a principal-branch change.
  constexpr double coordinate_tolerance = 1.0e-12;
  EXPECT_TRUE(
    logarithm.coordinates.isApprox(expected_coordinates, coordinate_tolerance));

  // Re-exponentiate the selected logarithm and compare the resulting finite
  // transform with the original one.
  const rigid_body_kinematics::Transform3 reconstructed =
    rigid_body_kinematics::Transform3::from_exponential_coordinates(
      logarithm.coordinates);

  const Eigen::Matrix4d original_matrix = original.matrix();
  const Eigen::Matrix4d reconstructed_matrix = reconstructed.matrix();

  // Keep the dimensionless rotation residual separate from the translation
  // residual measured in metres.

  const double rotation_error = (reconstructed_matrix.topLeftCorner<3, 3>() -
                                 original_matrix.topLeftCorner<3, 3>())
                                  .norm();

  const double translation_error =
    (reconstructed_matrix.topRightCorner<3, 1>() -
     original_matrix.topRightCorner<3, 1>())
      .norm();

  constexpr double round_trip_tolerance = 1.0e-10;
  EXPECT_LE(rotation_error, round_trip_tolerance);
  EXPECT_LE(translation_error, round_trip_tolerance);
}

TEST(Se3LogarithmTest, ReportsUnsupportedMagnitudeWhenLinearCoordinatesOverflow)
{
  const double largest_finite = std::numeric_limits<double>::max();

  // Use an exact half-turn about +z. Its inverse left Jacobian contains
  // off-diagonal factors of pi / 2, whose magnitude is greater than one.
  //
  // The transform translation remains finite, but applying J(phi)^{-1}
  // to it must overflow while recovering rho.
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    -1.0,  0.0, 0.0, 0.0,
     0.0, -1.0, 0.0, largest_finite,
     0.0,  0.0, 1.0, 0.0,
     0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  try {
    (void)transform.to_principal_exponential_coordinates();

    FAIL() << "Expected recovery of the linear coordinates to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(Se3LogarithmTest, SmallestPositiveTranslationIsPreservedAsPureTranslation)
{
  // denormal min: Smallest positive nonzero subnormal value
  const double smallest_positive = std::numeric_limits<double>::denorm_min();

  ASSERT_GT(smallest_positive, 0.0);

  // Use the smallest positive binary64 value as a translation in metres.
  // Although physically tiny, it is representably nonzero and must not be
  // classified as the complete identity.
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 3) = smallest_positive;

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::TransformLogResult actual =
    transform.to_principal_exponential_coordinates();

  rigid_body_kinematics::Vector6LinearFirst expected_coordinates =
    rigid_body_kinematics::Vector6LinearFirst::Zero();
  expected_coordinates(0) = smallest_positive;

  EXPECT_EQ(
    actual.branch, rigid_body_kinematics::TransformLogBranch::pure_translation);

  // The pure-translation logarithm performs no Jacobian calculation, so the
  // representable input value should be preserved exactly.
  EXPECT_EQ(actual.coordinates, expected_coordinates);
}

TEST(Se3LogarithmTest, RejectsOverlappingLogarithmBranchThresholds)
{
  rigid_body_kinematics::NumericalPolicy policy;

  // These values leave no nominal-angle interval because
  // series_angle_threshold >= pi - near_pi_tolerance.
  policy.series_angle_threshold = 2.0;
  policy.near_pi_tolerance = 2.0;

  const rigid_body_kinematics::Transform3 transform =
    rigid_body_kinematics::Transform3::identity();

  try {
    (void)transform.to_principal_exponential_coordinates(policy);

    FAIL() << "Expected overlapping logarithm branches to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::GeometryError::invalid_policy);
  }
}

TEST(Se3LogarithmTest, SeriesThresholdUsesInclusiveSmallAngleBranch)
{
  const rigid_body_kinematics::NumericalPolicy policy;
  const double threshold = policy.series_angle_threshold;

  struct BoundaryCase
  {
    double angle;
    rigid_body_kinematics::TransformLogBranch expected_branch;
  };

  // nextafter() selects the immediately adjacent representable double on
  // either side of the threshold.
  const std::array<BoundaryCase, 3> cases{
    BoundaryCase{
      std::nextafter(threshold, 0.0),
      rigid_body_kinematics::TransformLogBranch::small_angle,
    },
    BoundaryCase{
      threshold,
      rigid_body_kinematics::TransformLogBranch::small_angle,
    },
    BoundaryCase{
      std::nextafter(threshold, std::numeric_limits<double>::infinity()),
      rigid_body_kinematics::TransformLogBranch::nominal,
    },
  };

  for (const BoundaryCase & test_case : cases) {
    SCOPED_TRACE(test_case.angle);

    const rigid_body_kinematics::Vector6LinearFirst expected_coordinates(
      0.3, -0.2, 0.1, 0.0, 0.0, test_case.angle);

    const rigid_body_kinematics::Transform3 transform =
      rigid_body_kinematics::Transform3::from_exponential_coordinates(
        expected_coordinates, policy);

    const rigid_body_kinematics::TransformLogResult actual =
      transform.to_principal_exponential_coordinates(policy);

    EXPECT_EQ(actual.branch, test_case.expected_branch);

    constexpr double tolerance = 1.0e-12;
    EXPECT_TRUE(actual.coordinates.isApprox(expected_coordinates, tolerance));
  }
}

TEST(Se3LogarithmTest, NearPiThresholdUsesInclusiveNearPiBranch)
{
  const rigid_body_kinematics::NumericalPolicy policy;
  const double pi = std::acos(-1.0);
  const double near_pi_boundary = pi - policy.near_pi_tolerance;

  struct BoundaryCase
  {
    double angle;
    rigid_body_kinematics::TransformLogBranch expected_branch;
  };

  const std::array<BoundaryCase, 3> cases{
    BoundaryCase{
      std::nextafter(near_pi_boundary, 0.0),
      rigid_body_kinematics::TransformLogBranch::nominal,
    },
    BoundaryCase{
      near_pi_boundary,
      rigid_body_kinematics::TransformLogBranch::near_pi,
    },
    BoundaryCase{
      std::nextafter(near_pi_boundary, pi),
      rigid_body_kinematics::TransformLogBranch::near_pi,
    },
  };

  for (const BoundaryCase & test_case : cases) {
    SCOPED_TRACE(test_case.angle);

    const rigid_body_kinematics::Vector6LinearFirst expected_coordinates(
      0.3, -0.2, 0.1, 0.0, 0.0, test_case.angle);

    const rigid_body_kinematics::Transform3 transform =
      rigid_body_kinematics::Transform3::from_exponential_coordinates(
        expected_coordinates, policy);

    const rigid_body_kinematics::TransformLogResult actual =
      transform.to_principal_exponential_coordinates(policy);

    EXPECT_EQ(actual.branch, test_case.expected_branch);

    // The formulas on both sides of the branch boundary must recover the
    // represented coordinates without a discontinuity.
    constexpr double coordinate_tolerance = 1.0e-9;
    EXPECT_TRUE(
      actual.coordinates.isApprox(expected_coordinates, coordinate_tolerance));
  }
}
