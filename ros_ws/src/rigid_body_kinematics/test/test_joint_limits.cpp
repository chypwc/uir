#include <gtest/gtest.h>

#include <limits>
#include <optional>

#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/serial_chain_error.hpp"

TEST(JointLimitsTest, RevoluteIntervalMayExcludeZero)
{
  const rigid_body_kinematics::RevoluteLimits limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(0.2, 1.0);

  EXPECT_TRUE(limits.contains(0.2));
  EXPECT_TRUE(limits.contains(0.6));
  EXPECT_TRUE(limits.contains(1.0));

  EXPECT_FALSE(limits.contains(0.0));
  EXPECT_FALSE(limits.contains(0.1));
  EXPECT_FALSE(limits.contains(1.1));
}

TEST(JointLimitsTest, PrismaticOneSidedIntervalsAreUnbounded)
{
  const rigid_body_kinematics::PrismaticLimits lower_bounded =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-0.2, std::nullopt);

  EXPECT_TRUE(lower_bounded.contains(-0.2));
  EXPECT_TRUE(lower_bounded.contains(100.0));
  EXPECT_FALSE(lower_bounded.contains(-0.3));

  const rigid_body_kinematics::PrismaticLimits upper_bounded =
    rigid_body_kinematics::PrismaticLimits::from_bounds(std::nullopt, 0.5);

  EXPECT_TRUE(upper_bounded.contains(-100.0));
  EXPECT_TRUE(upper_bounded.contains(0.5));
  EXPECT_FALSE(upper_bounded.contains(0.6));
}

TEST(JointLimitsTest, AbsentRevoluteBoundsAcceptAnyFiniteAngle)
{
  const rigid_body_kinematics::RevoluteLimits limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(
      std::nullopt, std::nullopt);

  EXPECT_TRUE(limits.contains(-100000.0));
  EXPECT_TRUE(limits.contains(0.0));
  EXPECT_TRUE(limits.contains(100000.0));
}

TEST(JointLimitsTest, RejectsNaNRevoluteBound)
{
  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();

  try {
    (void)rigid_body_kinematics::RevoluteLimits::from_bounds(quiet_nan, 1.0);

    FAIL() << "Expected a NaN revolute bound to be rejected.";

  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::SerialChainError::invalid_joint_limits);
  }
}

TEST(JointLimitsTest, RejectsInfinitePrismaticBound)
{
  const double infinity = std::numeric_limits<double>::infinity();

  try {
    (void)rigid_body_kinematics::PrismaticLimits::from_bounds(-1.0, infinity);
    FAIL() << "Expected an infinite prismatic bound to be rejected.";
  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::SerialChainError::invalid_joint_limits);
  }
}

TEST(JointLimitsTest, RejectsLowerBoundAboveUpperBound)
{
  try {
    (void)rigid_body_kinematics::RevoluteLimits::from_bounds(1.0, -1.0);
    FAIL() << "Expected reversed joint-limit bounds to be rejected.";
  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(),
      rigid_body_kinematics::SerialChainError::invalid_joint_limits);
  }
}
