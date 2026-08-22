#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/lie_algebra.hpp"
#include "rigid_body_kinematics/transform3.hpp"

TEST(Se3AdjointTest, AcceptanceCaseMapsLinearFirstTwist)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 3) = 1.0;
  matrix(1, 3) = 2.0;
  matrix(2, 3) = 3.0;

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  // ξ_b = [ν_b; ω_b].
  const rigid_body_kinematics::Vector6LinearFirst twist_b(
    4.0, 5.0, 6.0, 1.0, 0.0, -1.0);

  const rigid_body_kinematics::Vector6LinearFirst twist_a =
    transform_ab.adjoint() * twist_b;

  // Since R = I, ω_a = ω_b and ν_a = ν_b + p x ω_b = [2; 9; 4].
  constexpr double tolerance = 1.0e-12;

  EXPECT_NEAR(twist_a(0), 2.0, tolerance);
  EXPECT_NEAR(twist_a(1), 9.0, tolerance);
  EXPECT_NEAR(twist_a(2), 4.0, tolerance);
  EXPECT_NEAR(twist_a(3), 1.0, tolerance);
  EXPECT_NEAR(twist_a(4), 0.0, tolerance);
  EXPECT_NEAR(twist_a(5), -1.0, tolerance);
}

TEST(Se3AdjointTest, RotationActsBeforeTranslationCrossProduct)
{
  // R_ab = R_z(pi/2), p_b^a = e_x.
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    0.0, -1.0, 0.0, 1.0,
    1.0,  0.0, 0.0, 0.0,
    0.0,  0.0, 1.0, 0.0,
    0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  // ξ_b = [ν_b; ω_b] = [e_x; e_y].
  const rigid_body_kinematics::Vector6LinearFirst twist_b(
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  const rigid_body_kinematics::Vector6LinearFirst twist_a =
    transform_ab.adjoint() * twist_b;

  constexpr double tolerance = 1.0e-12;

  // ξ_a = [R_ab ν_b + p_b^a x (R_ab ω_b); R_ab ω_b]
  // = [e_y + e_x x (-e_x); -e_x] = [e_y; -e_x].
  EXPECT_NEAR(twist_a(0), 0.0, tolerance);
  EXPECT_NEAR(twist_a(1), 1.0, tolerance);
  EXPECT_NEAR(twist_a(2), 0.0, tolerance);
  EXPECT_NEAR(twist_a(3), -1.0, tolerance);
  EXPECT_NEAR(twist_a(4), 0.0, tolerance);
  EXPECT_NEAR(twist_a(5), 0.0, tolerance);
}

TEST(Se3AdjointTest, SatisfiesHomogeneousConjugationIdentity)
{
  // Verify adjoint by hat(Ad_T ξ_b) = T_ab hat(ξ_b) T_ab^{-1}
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

  const rigid_body_kinematics::Vector6LinearFirst twist_b(
    0.4, -0.7, 1.2, 0.3, -0.5, 0.8);

  // Ad_T ξ_b
  const rigid_body_kinematics::Vector6LinearFirst twist_a =
    transform_ab.adjoint() * twist_b;

  // Left side: hat(Ad_T ξ_b).
  const Eigen::Matrix4d left = rigid_body_kinematics::hat_se3(twist_a);

  // Right side: T_ab hat(ξ_b) T_ab^{-1}.
  const Eigen::Matrix4d matrix_ba = transform_ab.inverse().matrix();
  const Eigen::Matrix4d right =
    matrix_ab * rigid_body_kinematics::hat_se3(twist_b) * matrix_ba;

  constexpr double tolerance = 1.0e-12;

  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      EXPECT_NEAR(left(row, column), right(row, column), tolerance);
    }
  }
}

TEST(Se3AdjointTest, IdentityTransformHasExactIdentityAdjoint)
{
  const rigid_body_kinematics::Transform3 identity_transform =
    rigid_body_kinematics::Transform3::identity();

  const Eigen::Matrix<double, 6, 6> actual = identity_transform.adjoint();

  const Eigen::Matrix<double, 6, 6> expected =
    Eigen::Matrix<double, 6, 6>::Identity();

  EXPECT_EQ(actual, expected);
}

TEST(Se3AdjointTest, ReportsUnsupportedMagnitudeWhenAdjointOverflows)
{
  const double largest_finite = std::numeric_limits<double>::max();
  const double cosine_quarter_turn = std::sqrt(0.5);

  // R_ab = R_x(pi/4) and p_b^a = [0, M, M]^T, where M is the largest
  // finite double. Every transform entry is finite.
  Eigen::Matrix4d matrix_ab;
  matrix_ab << 1.0, 0.0, 0.0, 0.0, 0.0, cosine_quarter_turn,
    -cosine_quarter_turn, largest_finite, 0.0, cosine_quarter_turn,
    cosine_quarter_turn, largest_finite, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 transform_ab =
    rigid_body_kinematics::Transform3::from_matrix(matrix_ab);

  try {
    (void)transform_ab.adjoint();
    FAIL() << "Expected the adjoint matrix calculation to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}
