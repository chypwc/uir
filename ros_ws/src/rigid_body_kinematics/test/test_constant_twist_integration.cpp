#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/numerical_policy.hpp"
#include "rigid_body_kinematics/transform3.hpp"

TEST(
  ConstantTwistIntegrationTest, ZeroDurationPreservesPoseForSpaceAndBodyTwists)
{
  // T_0 is deliberately non-identity so the test checks preservation of both
  // orientation and translation, not only the identity pose.
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    0.0, -1.0, 0.0, 1.0,
    1.0,  0.0, 0.0, 2.0,
    0.0,  0.0, 1.0, 3.0,
    0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  const rigid_body_kinematics::Vector6LinearFirst twist(
    1.0, -2.0, 3.0, 0.1, -0.2, 0.3);

  // The same numerical six-vector need not represent the same physical motion
  // in space and body coordinates.  That distinction is irrelevant here because
  // xi * 0 = 0 and exp(0) = I for either representation.
  const double delta_time = 0.0;

  const rigid_body_kinematics::Transform3 space_result =
    initial_transform.integrate_constant_space_twist(twist, delta_time);

  const rigid_body_kinematics::Transform3 body_result =
    initial_transform.integrate_constant_body_twist(twist, delta_time);

  EXPECT_EQ(space_result.matrix(), initial_transform.matrix());

  EXPECT_EQ(body_result.matrix(), initial_transform.matrix());
}

TEST(
  ConstantTwistIntegrationTest,
  EquivalentInitialSpaceAndBodyTwistsProduceSameAnalyticPose)
{
  // T_0 = T_sb(0): the body axes initially align with the space axes, and the
  // body origin O_b is one metre along +x_s from the space origin O_s.
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 3) = 1.0;

  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::from_matrix(matrix);

  // xi_b = [nu_b; ω_b].  The body origin moves at 1 m/s along +y_b while
  // the body rotates at 1 rad/s about +z_b.  Because the body axes rotate with
  // the circular motion, these body components remain constant.
  const rigid_body_kinematics::Vector6LinearFirst body_twist(
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  // At T_0, xi_s = Ad_T0 xi_b.  With R_0 = I and p_0 = e_x,
  // ν_s = ν_b + p_0 x ω_b = e_y + e_x x e_z = 0 and ω_s = e_z.
  // The zero spatial linear block is the velocity at O_s, not at O_b: this is
  // a pure space-referred rotation about O_s.
  const rigid_body_kinematics::Vector6LinearFirst space_twist(
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  const double pi = std::acos(-1.0);

  // Unit angular speed integrated for pi/2 seconds gives a quarter turn.  The
  // body origin therefore follows the unit circle from +x_s to +y_s.
  const double delta_time = 0.5 * pi;

  // The space increment acts on T_0 from the left; the body increment acts from
  // the right.  Equivalent twists must nevertheless produce the same pose.
  const rigid_body_kinematics::Transform3 space_result =
    initial_transform.integrate_constant_space_twist(space_twist, delta_time);

  const rigid_body_kinematics::Transform3 body_result =
    initial_transform.integrate_constant_body_twist(body_twist, delta_time);

  // The final rotation is R_z(pi/2), and O_b is at [0, 1, 0]^T metres.  This
  // analytic matrix is constructed without either production integration path.
  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
    0.0, -1.0, 0.0, 0.0,
    1.0,  0.0, 0.0, 1.0,
    0.0,  0.0, 1.0, 0.0,
    0.0,  0.0, 0.0, 1.0;
  // clang-format on

  constexpr double rotation_tolerance = 1.0e-12;
  constexpr double translation_tolerance = 1.0e-12;  // metres

  const Eigen::Matrix4d actual_space = space_result.matrix();
  const Eigen::Matrix4d actual_body = body_result.matrix();

  const Eigen::Matrix3d expected_rotation = expected.block<3, 3>(0, 0);
  const Eigen::Vector3d expected_translation = expected.block<3, 1>(0, 3);

  const Eigen::Matrix3d actual_space_rotation = actual_space.block<3, 3>(0, 0);
  const Eigen::Vector3d actual_space_translation =
    actual_space.block<3, 1>(0, 3);

  const Eigen::Matrix3d actual_body_rotation = actual_body.block<3, 3>(0, 0);
  const Eigen::Vector3d actual_body_translation = actual_body.block<3, 1>(0, 3);

  EXPECT_TRUE(
    actual_space_rotation.isApprox(expected_rotation, rotation_tolerance));
  EXPECT_TRUE(actual_space_translation.isApprox(
    expected_translation, translation_tolerance));

  EXPECT_TRUE(
    actual_body_rotation.isApprox(expected_rotation, rotation_tolerance));
  EXPECT_TRUE(actual_body_translation.isApprox(
    expected_translation, translation_tolerance));
}

TEST(ConstantTwistIntegrationTest, NegativeDurationIsRejectedForSpaceTwist)
{
  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  const rigid_body_kinematics::Vector6LinearFirst space_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  const double negative_duration = -1.0;  // seconds

  try {
    (void)initial_transform.integrate_constant_space_twist(
      space_twist, negative_duration);

    FAIL() << "Expected a negative integration duration to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::invalid_time);
  }
}

TEST(ConstantTwistIntegrationTest, NegativeDurationIsRejectedForBodyTwist)
{
  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  const rigid_body_kinematics::Vector6LinearFirst body_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  const double negative_duration = -1.0;  // seconds

  try {
    (void)initial_transform.integrate_constant_body_twist(
      body_twist, negative_duration);

    FAIL() << "Expected a negative integration duration to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::invalid_time);
  }
}

TEST(ConstantTwistIntegrationTest, NonFiniteSpaceTwistIsRejected)
{
  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  rigid_body_kinematics::Vector6LinearFirst space_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  space_twist(0) = std::numeric_limits<double>::infinity();

  const double delta_time = 1.0;  // seconds

  try {
    (void)initial_transform.integrate_constant_space_twist(
      space_twist, delta_time);

    FAIL() << "Expected a non-finite space twist to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(ConstantTwistIntegrationTest, NonFiniteBodyTwistIsRejected)
{
  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  rigid_body_kinematics::Vector6LinearFirst body_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  body_twist(5) = std::numeric_limits<double>::infinity();

  const double delta_time = 1.0;  // seconds

  try {
    (void)initial_transform.integrate_constant_body_twist(
      body_twist, delta_time);

    FAIL() << "Expected a non-finite body twist to be rejected.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(error.code(), rigid_body_kinematics::GeometryError::non_finite);
  }
}

TEST(ConstantTwistIntegrationTest, NonFiniteDurationIsRejectedForSpaceTwist)
{
  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  const rigid_body_kinematics::Vector6LinearFirst space_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  const double invalid_durations[] = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()};

  for (const double invalid_duration : invalid_durations) {
    SCOPED_TRACE(
      ::testing::Message() << "invalid_duration = " << invalid_duration);

    try {
      (void)initial_transform.integrate_constant_space_twist(
        space_twist, invalid_duration);

      FAIL() << "Expected a non-finite duration to be rejected.";
    } catch (const rigid_body_kinematics::GeometryException & error) {
      EXPECT_EQ(
        error.code(), rigid_body_kinematics::GeometryError::invalid_time);
    }
  }
}

TEST(ConstantTwistIntegrationTest, NonFiniteDurationIsRejectedForBodyTwist)
{
  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  const rigid_body_kinematics::Vector6LinearFirst body_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  const double invalid_durations[] = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()};

  for (const double invalid_duration : invalid_durations) {
    SCOPED_TRACE(
      ::testing::Message() << "invalid_duration = " << invalid_duration);

    try {
      (void)initial_transform.integrate_constant_body_twist(
        body_twist, invalid_duration);

      FAIL() << "Expected a non-finite duration to be rejected.";
    } catch (const rigid_body_kinematics::GeometryException & error) {
      EXPECT_EQ(
        error.code(), rigid_body_kinematics::GeometryError::invalid_time);
    }
  }
}

TEST(
  ConstantTwistIntegrationTest,
  ReportsUnsupportedMagnitudeWhenSpaceTwistTimesDurationOverflows)
{
  rigid_body_kinematics::Vector6LinearFirst space_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  // The input velocity is finite, but multiplying it by two seconds exceeds
  // the largest representable double.
  space_twist(0) = std::numeric_limits<double>::max();

  const double delta_time = 2.0;  // seconds

  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  try {
    (void)initial_transform.integrate_constant_space_twist(
      space_twist, delta_time);

    FAIL() << "Expected the exponential coordinates to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}

TEST(
  ConstantTwistIntegrationTest,
  ReportsUnsupportedMagnitudeWhenBodyTwistTimesDurationOverflows)
{
  rigid_body_kinematics::Vector6LinearFirst body_twist =
    rigid_body_kinematics::Vector6LinearFirst::Zero();

  // The input velocity is finite, but multiplying it by two seconds exceeds
  // the largest representable double.
  body_twist(0) = std::numeric_limits<double>::max();

  const double delta_time = 2.0;  // seconds

  const rigid_body_kinematics::Transform3 initial_transform =
    rigid_body_kinematics::Transform3::identity();

  try {
    (void)initial_transform.integrate_constant_body_twist(
      body_twist, delta_time);

    FAIL() << "Expected the exponential coordinates to overflow.";
  } catch (const rigid_body_kinematics::GeometryException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::GeometryError::unsupported_magnitude);
  }
}
