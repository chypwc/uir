#include <gtest/gtest.h>

#include <Eigen/Core>
#include <limits>

#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/serial_chain_error.hpp"

TEST(
  JointDefinitionTest, NormalizesDirectionAndDerivesDisplacedRevoluteScrewAxis)
{
  const Eigen::Vector3d space_axis_direction(0.0, 0.0, 5.0);
  const Eigen::Vector3d space_axis_point_metres(2.0, 0.0, 0.0);

  const rigid_body_kinematics::RevoluteLimits limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(-2.0, 2.0);

  const rigid_body_kinematics::RevoluteJoint joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      space_axis_direction, space_axis_point_metres, limits);

  const Eigen::Vector3d expected_direction(0.0, 0.0, 1.0);

  rigid_body_kinematics::Vector6LinearFirst expected_screw_axis;
  expected_screw_axis << 0.0, -2.0, 0.0, 0.0, 0.0, 1.0;

  constexpr double dimensionless_tolerance = 1.0e-12;
  constexpr double linear_tolerance_metres = 1.0e-12;

  for (Eigen::Index index = 0; index < 3; ++index) {
    EXPECT_NEAR(
      joint.space_axis_direction()(index), expected_direction(index),
      dimensionless_tolerance);

    EXPECT_NEAR(
      joint.space_screw_axis()(index), expected_screw_axis(index),
      linear_tolerance_metres);
  }

  for (Eigen::Index index = 3; index < 6; ++index) {
    EXPECT_NEAR(
      joint.space_screw_axis()(index), expected_screw_axis(index),
      dimensionless_tolerance);
  }
}

TEST(JointDefinitionTest, NormalizesDirectionAndDerivesPrismaticsScrewAxis)
{
  const Eigen::Vector3d space_axis_direction(0.0, -4.0, 0.0);

  const rigid_body_kinematics::PrismaticLimits limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-0.5, 1.0);

  const rigid_body_kinematics::PrismaticJoint joint =
    rigid_body_kinematics::PrismaticJoint::from_axis(
      space_axis_direction, limits);

  const Eigen::Vector3d expected_direction(0.0, -1.0, 0.0);

  rigid_body_kinematics::Vector6LinearFirst expected_screw_axis;
  expected_screw_axis << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0;

  constexpr double dimensionless_tolerance = 1.0e-12;

  for (Eigen::Index index = 0; index < 3; ++index) {
    EXPECT_NEAR(
      joint.space_axis_direction()(index), expected_direction(index),
      dimensionless_tolerance);
  }

  for (Eigen::Index index = 0; index < 6; ++index) {
    EXPECT_NEAR(
      joint.space_screw_axis()(index), expected_screw_axis(index),
      dimensionless_tolerance);
  }
}

TEST(JointDefinitionTest, RejectsZeroPrismaticAxisDirection)
{
  const Eigen::Vector3d zero_direction = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::PrismaticLimits limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-0.5, 1.0);

  try {
    (void)rigid_body_kinematics::PrismaticJoint::from_axis(
      zero_direction, limits);

    FAIL() << "Expected a zero prismatic axis direction to be rejected.";
  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::SerialChainError::invalid_model);
  }
}

TEST(JointDefinitionTest, RejectsNonFiniteRevoluteAxisDirection)
{
  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();

  const Eigen::Vector3d non_finite_direction(quiet_nan, 0.0, 1.0);
  const Eigen::Vector3d axis_point_metres = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::RevoluteLimits limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(-1.0, 1.0);
  try {
    (void)rigid_body_kinematics::RevoluteJoint::from_axis(
      non_finite_direction, axis_point_metres, limits);
    FAIL() << "Expected a non-finite revolute axis direction to be rejected.";
  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::SerialChainError::non_finite);
  }
}

TEST(JointDefinitionTest, RejectsNonFiniteRevoluteAxisPoint)
{
  const double infinity = std::numeric_limits<double>::infinity();

  const Eigen::Vector3d axis_direction(0.0, 0.0, 1.0);
  const Eigen::Vector3d non_finite_axis_point_metres(infinity, 0.0, 0.0);

  const rigid_body_kinematics::RevoluteLimits limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(-1.0, 1.0);

  try {
    (void)rigid_body_kinematics::RevoluteJoint::from_axis(
      axis_direction, non_finite_axis_point_metres, limits);

    FAIL() << "Expected a non-finite revolute axis point to be rejected.";
  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::SerialChainError::non_finite);
  }
}

TEST(JointDefinitionTest, PointsOnSameRevoluteAxisProduceSameScrewAxis)
{
  const Eigen::Vector3d axis_direction(0.0, 0.0, 2.0);

  const Eigen::Vector3d first_axis_point_metres(2.0, 0.0, 0.0);
  const Eigen::Vector3d second_axis_point_metres(2.0, 0.0, 7.0);

  const rigid_body_kinematics::RevoluteLimits limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(-1.0, 1.0);

  const rigid_body_kinematics::RevoluteJoint first_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      axis_direction, first_axis_point_metres, limits);

  const rigid_body_kinematics::RevoluteJoint second_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      axis_direction, second_axis_point_metres, limits);

  constexpr double linear_tolerance_metres = 1.0e-12;
  constexpr double dimensionless_tolerance = 1.0e-12;

  for (Eigen::Index index = 0; index < 3; ++index) {
    EXPECT_NEAR(
      first_joint.space_screw_axis()(index),
      second_joint.space_screw_axis()(index), linear_tolerance_metres);
  }

  for (Eigen::Index index = 3; index < 6; ++index) {
    EXPECT_NEAR(
      first_joint.space_screw_axis()(index),
      second_joint.space_screw_axis()(index), dimensionless_tolerance);
  }
}
