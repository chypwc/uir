#include <gtest/gtest.h>

#include <Eigen/Core>
#include <variant>
#include <vector>

#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/serial_chain_error.hpp"
#include "rigid_body_kinematics/serial_chain_model.hpp"
#include "rigid_body_kinematics/transform3.hpp"

TEST(SerialChainModelTest, StoresHomePoseAndOneJoint)
{
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = 1.0;
  home_matrix(1, 3) = 2.0;
  home_matrix(2, 3) = 3.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  const rigid_body_kinematics::PrismaticLimits limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-0.5, 1.0);

  const rigid_body_kinematics::PrismaticJoint joint =
    rigid_body_kinematics::PrismaticJoint::from_axis(
      Eigen::Vector3d(1.0, 0.0, 0.0), limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    joint};

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  EXPECT_EQ(model.joint_count(), 1U);

  ASSERT_EQ(model.joint_definitions().size(), 1U);

  EXPECT_TRUE(
    std::holds_alternative<rigid_body_kinematics::PrismaticJoint>(
      model.joint_definitions().front()));

  constexpr double tolerance = 1.0e-12;
  EXPECT_TRUE(model.home_pose().matrix().isApprox(home_matrix, tolerance));
}

TEST(SerialChainModelTest, RejectsEmptyJointList)
{
  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::identity();

  const std::vector<rigid_body_kinematics::JointDefinition>
    empty_joint_definitions;

  try {
    (void)rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, empty_joint_definitions);

    FAIL() << "Expected an empty serial-chain model to be rejected.";
  } catch (const rigid_body_kinematics::SerialChainException & error) {
    EXPECT_EQ(
      error.code(), rigid_body_kinematics::SerialChainError::invalid_model);
  }
}

TEST(SerialChainModelTest, PreservesMixedJointOrder)
{
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(-1.0, 1.0);

  const rigid_body_kinematics::PrismaticLimits prismatic_limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-0.5, 0.5);

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), revolute_limits);

  const rigid_body_kinematics::PrismaticJoint prismatic_joint =
    rigid_body_kinematics::PrismaticJoint::from_axis(
      Eigen::Vector3d(1.0, 0.0, 0.0), prismatic_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint, prismatic_joint};

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      rigid_body_kinematics::Transform3::identity(), joint_definitions);

  ASSERT_EQ(model.joint_count(), 2U);

  EXPECT_TRUE(
    std::holds_alternative<rigid_body_kinematics::RevoluteJoint>(
      model.joint_definitions().at(0)));

  EXPECT_TRUE(
    std::holds_alternative<rigid_body_kinematics::PrismaticJoint>(
      model.joint_definitions().at(1)));
}
