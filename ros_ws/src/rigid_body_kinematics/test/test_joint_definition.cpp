#include <gtest/gtest.h>

#include <Eigen/Core>

#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"

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
