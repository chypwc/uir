#include <gtest/gtest.h>

#include <Eigen/Core>
#include <numbers>
#include <vector>

#include "rigid_body_kinematics/forward_kinematics.hpp"

namespace
{
constexpr double kTranslationToleranceMetres = 1.0e-12;
constexpr double kRotationToleranceRadians = 1.0e-12;

void expect_pose_near(
  const rigid_body_kinematics::Transform3 & expected,
  const rigid_body_kinematics::Transform3 & actual,
  double translation_tolerance_metres, double rotation_tolerance_radians)
{
  const rigid_body_kinematics::Transform3 error_pose =
    expected.inverse().compose(actual);

  const Eigen::Matrix4d error_matrix = error_pose.matrix();

  const Eigen::Vector3d translation_error_metres =
    error_matrix.block<3, 1>(0, 3);

  const rigid_body_kinematics::TransformLogResult error_log =
    error_pose.to_principal_exponential_coordinates();

  const Eigen::Vector3d rotation_error_radians =
    error_log.coordinates.tail<3>();

  EXPECT_LE(translation_error_metres.norm(), translation_tolerance_metres);
  EXPECT_LE(rotation_error_radians.norm(), rotation_tolerance_radians);
}
}  // namespace

TEST(ForwardKinematicsTest, ZeroJointCoordinatesReturnHomePose)
{
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(-1.0, 2.0);

  const rigid_body_kinematics::PrismaticLimits prismatic_limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-1.0, 5.0);

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      revolute_limits);

  const rigid_body_kinematics::PrismaticJoint prismatic_joint =
    rigid_body_kinematics::PrismaticJoint::from_axis(
      Eigen::Vector3d(0.0, 0.0, 1.0), prismatic_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint, prismatic_joint};

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_exponential_coordinates(
      rigid_body_kinematics::Vector6LinearFirst(1.0, 0.0, 0.0, 0.0, 0.0, 1.0));

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  const rigid_body_kinematics::Transform3 actual =
    rigid_body_kinematics::space_form_forward_kinematics(
      model, Eigen::Vector2d::Zero());

  expect_pose_near(
    home_pose, actual, kTranslationToleranceMetres, kRotationToleranceRadians);
}

TEST(ForwardKinematicsTest, RevoluteJointRotatesHomePoseAboutSpaceOrigin)
{
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(
      -std::numbers::pi, std::numbers::pi);

  // Rotate aboout +z axis.
  const Eigen::Vector3d space_axis_direction(0.0, 0.0, 5.0);
  const Eigen::Vector3d space_axis_point_metres = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      space_axis_direction, space_axis_point_metres, revolute_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint};

  // Home pose is at (2, 0, 0) without orientation.
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = 2.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  // Rotate pi/2 abouot +z axis.
  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << std::numbers::pi / 2.0;

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  const rigid_body_kinematics::Transform3 actual =
    rigid_body_kinematics::space_form_forward_kinematics(
      model, joint_coordinates);

  // After rotation: (2, 0, 0) -> (0, 2, 0)
  Eigen::Matrix4d expected_matrix;
  // clang-format off
  expected_matrix <<
      0.0, -1.0, 0.0, 0.0,
      1.0,  0.0, 0.0, 2.0,
      0.0,  0.0, 1.0, 0.0,
      0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 expected_pose =
    rigid_body_kinematics::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected_pose, actual, kTranslationToleranceMetres,
    kRotationToleranceRadians);
}

TEST(ForwardKinematicsTest, DisplacedRevoluteAxisRotatesHomePoseAboutAxisLine)
{
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(
      -std::numbers::pi, std::numbers::pi);

  // Rotate about +z axis: s_ω.
  const Eigen::Vector3d space_axis_direction(0.0, 0.0, 1.0);
  // A point on the s_ω.
  const Eigen::Vector3d space_axis_point_metres(2.0, 0.0, 0.0);

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      space_axis_direction, space_axis_point_metres, revolute_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint};

  // Suppose initial position (3, 0, 0).
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = 3.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  // q_1 = π / 2.
  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << std::numbers::pi / 2.0;

  const rigid_body_kinematics::Transform3 actual =
    rigid_body_kinematics::space_form_forward_kinematics(
      model, joint_coordinates);

  // (3, 0, 0) rotates about +z axis on (2, 0, 0).
  // -> R_z(π/2) [(3, 0, 0) - (2, 0, 0)] + (2, 0, 0)
  // = (0, 1, 0) + (2, 0, 0) = (2, 1, 0)
  Eigen::Matrix4d expected_matrix;
  // clang-format off
  expected_matrix <<
      0.0, -1.0, 0.0, 2.0,
      1.0,  0.0, 0.0, 1.0,
      0.0,  0.0, 1.0, 0.0,
      0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 expected_pose =
    rigid_body_kinematics::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected_pose, actual, kTranslationToleranceMetres,
    kRotationToleranceRadians);
}

TEST(ForwardKinematicsTest, PrismaticJointTranslatesAlongNormalizedSpaceAxis)
{
  // Construct the joint and home pose.

  const rigid_body_kinematics::PrismaticLimits prismatic_limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-10.0, 10.0);

  const Eigen::Vector3d space_axis_direction(0.0, -4.0, 0.0);

  const rigid_body_kinematics::PrismaticJoint prismatic_joint =
    rigid_body_kinematics::PrismaticJoint::from_axis(
      space_axis_direction, prismatic_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    prismatic_joint};

  // Suppose initial position (1, 2, 3).
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix.block<3, 1>(0, 3) << 1.0, 2.0, 3.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  // Evaluate a displacement of 0.4 metres.
  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << 0.4;

  const rigid_body_kinematics::Transform3 actual =
    rigid_body_kinematics::space_form_forward_kinematics(
      model, joint_coordinates);

  // Compare with the independent expected pose.
  // (1, 2, 3) + 0.4 (0, -1, 0) = (1, 1.6, 3)
  Eigen::Matrix4d expected_matrix;
  // clang-format off
  expected_matrix <<
      1.0, 0.0, 0.0, 1.0,
      0.0, 1.0, 0.0, 1.6,
      0.0, 0.0, 1.0, 3.0,
      0.0, 0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 expected_pose =
    rigid_body_kinematics::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected_pose, actual, kTranslationToleranceMetres,
    kRotationToleranceRadians);
}

TEST(ForwardKinematicsTest, RevoluteJointRotatesHomePoseAboutSpaceXAxis)
{
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(
      -std::numbers::pi, std::numbers::pi);

  // Rotate about +x axis: s_ω.
  const Eigen::Vector3d space_axis_direction(1.0, 0.0, 0.0);
  // A point on the s_ω.
  const Eigen::Vector3d space_axis_point_metres = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      space_axis_direction, space_axis_point_metres, revolute_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint};

  // Suppose initial position (0, 1, 0).
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(1, 3) = 1.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  // q_1 = π / 2.
  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << std::numbers::pi / 2.0;

  const rigid_body_kinematics::Transform3 actual =
    rigid_body_kinematics::space_form_forward_kinematics(
      model, joint_coordinates);

  // (0, 1, 0) rotates about +x axis on (0, 0, 0).
  // -> R_x(π/2) (0, 1, 0) = (0, 0, 1)
  Eigen::Matrix4d expected_matrix;
  // clang-format off
  expected_matrix <<
      1.0, 0.0,  0.0, 0.0,
      0.0, 0.0, -1.0, 0.0,
      0.0, 1.0,  0.0, 1.0,
      0.0, 0.0,  0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 expected_pose =
    rigid_body_kinematics::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected_pose, actual, kTranslationToleranceMetres,
    kRotationToleranceRadians);
}

TEST(ForwardKinematicsTest, MixedChainPreservesRevolutePrismaticOrder)
{
  // Revolute joint.
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(
      -std::numbers::pi, std::numbers::pi);

  // Rotate about +z axis through the space-frame origin.
  const Eigen::Vector3d space_axis_direction_revolute(0.0, 0.0, 1.0);
  const Eigen::Vector3d space_axis_point_metres = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      space_axis_direction_revolute, space_axis_point_metres, revolute_limits);

  // Prismatic joint.
  const rigid_body_kinematics::PrismaticLimits prismatic_limits =
    rigid_body_kinematics::PrismaticLimits::from_bounds(-10.0, 10.0);

  const Eigen::Vector3d space_axis_direction_prismatic(1.0, 0.0, 0.0);

  const rigid_body_kinematics::PrismaticJoint prismatic_joint =
    rigid_body_kinematics::PrismaticJoint::from_axis(
      space_axis_direction_prismatic, prismatic_limits);

  // prismatic_joint applies on home_pose first.
  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint, prismatic_joint};

  // Suppose initial position (1, 0, 0).
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = 1.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  const rigid_body_kinematics::SerialChainModel model =
    rigid_body_kinematics::SerialChainModel::from_home_and_joints(
      home_pose, joint_definitions);

  // q_1 = pi/2 radians; q_2 = 0.5 metres.
  Eigen::VectorXd joint_coordinates(2);
  joint_coordinates << std::numbers::pi / 2.0, 0.5;

  const rigid_body_kinematics::Transform3 actual =
    rigid_body_kinematics::space_form_forward_kinematics(
      model, joint_coordinates);

  // R_z(π/2) [(1, 0, 0) + (0.5, 0, 0)] = (0, 1.5, 0)
  Eigen::Matrix4d expected_matrix;
  // clang-format off
  expected_matrix <<
      0.0, -1.0, 0.0, 0.0,
      1.0,  0.0, 0.0, 1.5,
      0.0,  0.0, 1.0, 0.0,
      0.0,  0.0, 0.0, 1.0;
  // clang-format on

  const rigid_body_kinematics::Transform3 expected_pose =
    rigid_body_kinematics::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected_pose, actual, kTranslationToleranceMetres,
    kRotationToleranceRadians);
}
