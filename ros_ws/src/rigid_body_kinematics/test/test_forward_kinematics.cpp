#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include "rigid_body_kinematics/forward_kinematics.hpp"
#include "rigid_body_kinematics/serial_chain_error.hpp"

// T(q) = E_1(q_1) ... E_n(q_n) M, with E_j = exp(hat(S_j q_j)).
// Screws are linear-first and expressed in the fixed space frame {s}.
// Positions are in metres; revolute coordinates are in radians.
namespace
{
constexpr double kTranslationToleranceMetres = 1.0e-12;
constexpr double kRotationToleranceRadians = 1.0e-12;

// Compare T_error = T_expected^{-1} T_actual: its translation norm is in m,
// and the angular part of log(T_error) has a norm in rad. Keep them separate.
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

// Purpose: Preserve the complete home pose when all allowed coordinates are zero.
// E_j(0) = I_4, so T(0) = I_4 ... I_4 M = M, including its rotation.
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

// Purpose: Verify positive rotation and axis normalization at the space origin.
// Normalize (0, 0, 5) to e_z; R = R_z(pi/2) and p = R (2, 0, 0) = (0, 2, 0) m.
TEST(ForwardKinematicsTest, RevoluteJointRotatesHomePoseAboutSpaceOrigin)
{
  const rigid_body_kinematics::RevoluteLimits revolute_limits =
    rigid_body_kinematics::RevoluteLimits::from_bounds(
      -std::numbers::pi, std::numbers::pi);

  // Rotate about the positive space z-axis.
  const Eigen::Vector3d space_axis_direction(0.0, 0.0, 5.0);
  const Eigen::Vector3d space_axis_point_metres = Eigen::Vector3d::Zero();

  const rigid_body_kinematics::RevoluteJoint revolute_joint =
    rigid_body_kinematics::RevoluteJoint::from_axis(
      space_axis_direction, space_axis_point_metres, revolute_limits);

  const std::vector<rigid_body_kinematics::JointDefinition> joint_definitions{
    revolute_joint};

  // Home pose: position (2, 0, 0) m and identity orientation.
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = 2.0;

  const rigid_body_kinematics::Transform3 home_pose =
    rigid_body_kinematics::Transform3::from_matrix(home_matrix);

  // Rotate pi/2 radians about the positive space z-axis.
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

// Purpose: Verify rotation about an offset axis, including the sign of -s x a.
// For axis point a = (2, 0, 0) m, p = R_z(pi/2) (p_0 - a) + a = (2, 1, 0) m.
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

// Purpose: Verify signed-axis translation without changing orientation.
// With s = (0, -1, 0), p = p_0 + q s = (1, 1.6, 3) m and R = I_3.
// The supplied direction's length must not scale the 0.4 m displacement.
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

// Purpose: Exercise spatial motion outside the space x-y plane.
// R = R_x(pi/2) sends p_0 = (0, 1, 0) m to p = (0, 0, 1) m.
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

// Purpose: Verify ordered composition of revolute and prismatic motion.
// T = E_R E_P M gives p = R_z(pi/2) ((1, 0, 0) + (0.5, 0, 0))
// = (0, 1.5, 0) m; the proximal rotation carries the distal extension.
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

  // Store base-to-end-effector order; E_R E_P M acts rightmost first.
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

// Purpose: Check two revolute joints against independent link geometry.
// For lengths 2 m and 1 m: x = 2 cos(q_1) + cos(q_1 + q_2),
// y = 2 sin(q_1) + sin(q_1 + q_2), and R = R_z(q_1 + q_2).
// Angles (pi/2, -pi/2) rad give p = (1, 2, 0) m and R = I_3.
TEST(ForwardKinematicsTest, PlanarTwoRevoluteChainMatchesAnalyticPose)
{
  namespace rbk = rigid_body_kinematics;

  const rbk::RevoluteLimits limits =
    rbk::RevoluteLimits::from_bounds(-std::numbers::pi, std::numbers::pi);

  // Both axes point along +z at the zero arrangement.
  const Eigen::Vector3d axis_direction(0.0, 0.0, 1.0);

  const rbk::RevoluteJoint joint_1 = rbk::RevoluteJoint::from_axis(
    axis_direction, Eigen::Vector3d::Zero(), limits);

  const rbk::RevoluteJoint joint_2 = rbk::RevoluteJoint::from_axis(
    axis_direction, Eigen::Vector3d(2.0, 0.0, 0.0), limits);

  // Link lengths: 2 m and 1 m; both extend along +x at zero.
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = 3.0;

  const rbk::Transform3 home_pose = rbk::Transform3::from_matrix(home_matrix);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(home_pose, {joint_1, joint_2});

  Eigen::VectorXd joint_coordinates(2);
  joint_coordinates << std::numbers::pi / 2.0, -std::numbers::pi / 2.0;

  const rbk::Transform3 actual =
    rbk::space_form_forward_kinematics(model, joint_coordinates);

  // Independent link geometry:
  // x = 2 cos(pi/2) + cos(0) = 1 m
  // y = 2 sin(pi/2) + sin(0) = 2 m
  // Orientation: pi/2 - pi/2 = 0 rad.
  Eigen::Matrix4d expected_matrix = Eigen::Matrix4d::Identity();
  expected_matrix(0, 3) = 1.0;
  expected_matrix(1, 3) = 2.0;

  const rbk::Transform3 expected =
    rbk::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected, actual, kTranslationToleranceMetres, kRotationToleranceRadians);
}

// Purpose: Enforce one coordinate per joint: q must belong to R^n.
// For n = 2, lengths 1 and 3 must throw dimension_mismatch, not return a pose.
TEST(ForwardKinematicsTest, RejectsWrongJointCoordinateCount)
{
  namespace rbk = rigid_body_kinematics;

  // Valid two-joint model; zero is within both joint intervals.
  const rbk::PrismaticLimits limits =
    rbk::PrismaticLimits::from_bounds(-1.0, 1.0);

  const rbk::PrismaticJoint joint_1 =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(1.0, 0.0, 0.0), limits);

  const rbk::PrismaticJoint joint_2 =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(0.0, 1.0, 0.0), limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {joint_1, joint_2});

  // Two coordinates are required: test too few and too many.
  for (const int coordinate_count : {1, 3}) {
    const Eigen::VectorXd joint_coordinates =
      Eigen::VectorXd::Zero(coordinate_count);

    try {
      static_cast<void>(
        rbk::space_form_forward_kinematics(model, joint_coordinates));

      FAIL() << "Expected dimension_mismatch for " << coordinate_count
             << " coordinates.";
    } catch (const rbk::SerialChainException & error) {
      EXPECT_EQ(error.code(), rbk::SerialChainError::dimension_mismatch)
        << "Coordinate count: " << coordinate_count;
    }
  }
}

// Purpose: Reject NaN and +/-infinity as supplied data, not as limit violations.
// The second coordinate is invalid while the first is finite; this checks
// validation beyond the first entry and requires the non_finite error code.
TEST(ForwardKinematicsTest, RejectsNonFiniteJointCoordinates)
{
  namespace rbk = rigid_body_kinematics;

  const rbk::PrismaticLimits limits =
    rbk::PrismaticLimits::from_bounds(-1.0, 1.0);

  const rbk::PrismaticJoint joint_1 =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(1.0, 0.0, 0.0), limits);

  const rbk::PrismaticJoint joint_2 =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(0.0, 1.0, 0.0), limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {joint_1, joint_2});

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  for (const double invalid_coordinate : {nan, infinity, -infinity}) {
    // Keep joint 1 valid; place the invalid value at joint 2.
    Eigen::VectorXd joint_coordinates(2);
    joint_coordinates << 0.0, invalid_coordinate;

    try {
      static_cast<void>(
        rbk::space_form_forward_kinematics(model, joint_coordinates));

      FAIL() << "Expected non_finite for coordinate " << invalid_coordinate;
    } catch (const rbk::SerialChainException & error) {
      EXPECT_EQ(error.code(), rbk::SerialChainError::non_finite)
        << "Invalid coordinate: " << invalid_coordinate;
    }
  }
}

// Purpose: Enforce exact membership q_j in [0.5, 1.5] for both joint types.
// Endpoints succeed; zero and the nearest values outside the bounds fail.
// Revolute bounds are in rad and prismatic bounds in m; no clamping is allowed.
TEST(ForwardKinematicsTest, RespectsInclusiveJointLimits)
{
  namespace rbk = rigid_body_kinematics;

  const rbk::RevoluteLimits revolute_limits =
    rbk::RevoluteLimits::from_bounds(0.5, 1.5);

  const rbk::PrismaticLimits prismatic_limits =
    rbk::PrismaticLimits::from_bounds(0.5, 1.5);

  const rbk::RevoluteJoint revolute_joint = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), revolute_limits);

  const rbk::PrismaticJoint prismatic_joint = rbk::PrismaticJoint::from_axis(
    Eigen::Vector3d(1.0, 0.0, 0.0), prismatic_limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {revolute_joint, prismatic_joint});

  const double just_below_lower = std::nextafter(0.5, 0.0);
  const double just_above_upper = std::nextafter(1.5, 2.0);

  // Exercise each joint's limits while keeping the other coordinate valid.
  for (const int joint_index : {0, 1}) {
    Eigen::VectorXd joint_coordinates = Eigen::VectorXd::Ones(2);

    for (const double endpoint : {0.5, 1.5}) {
      joint_coordinates(joint_index) = endpoint;

      EXPECT_NO_THROW(static_cast<void>(
        rbk::space_form_forward_kinematics(model, joint_coordinates)))
        << "Joint " << joint_index + 1 << ", endpoint " << endpoint;
    }

    for (const double outside_coordinate :
         {0.0, just_below_lower, just_above_upper}) {
      joint_coordinates(joint_index) = outside_coordinate;

      try {
        static_cast<void>(
          rbk::space_form_forward_kinematics(model, joint_coordinates));

        FAIL() << "Expected joint_out_of_domain for joint " << joint_index + 1
               << ", coordinate " << outside_coordinate;
      } catch (const rbk::SerialChainException & error) {
        EXPECT_EQ(error.code(), rbk::SerialChainError::joint_out_of_domain)
          << "Joint " << joint_index + 1 << ", coordinate "
          << outside_coordinate;
      }
    }
  }
}

// Purpose: Reject an invalid policy even for the zero-motion query T(0) = M.
// The finite thresholds must satisfy 0 < series_angle_threshold < pi
// and maximum_exponential_angle >= pi; violations require invalid_policy.
TEST(ForwardKinematicsTest, RejectsInvalidExponentialPolicy)
{
  namespace rbk = rigid_body_kinematics;

  const rbk::PrismaticLimits limits =
    rbk::PrismaticLimits::from_bounds(-1.0, 1.0);

  const rbk::PrismaticJoint joint =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(1.0, 0.0, 0.0), limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {joint});

  const Eigen::VectorXd joint_coordinates = Eigen::VectorXd::Zero(1);

  // Reuse the same exception check for each invalid policy.
  const auto expect_invalid_policy =
    [&model, &joint_coordinates](const rbk::NumericalPolicy & policy) {
      try {
        static_cast<void>(
          rbk::space_form_forward_kinematics(model, joint_coordinates, policy));

        FAIL() << "Expected invalid_policy; series threshold = "
               << policy.series_angle_threshold
               << ", maximum angle = " << policy.maximum_exponential_angle;
      } catch (const rbk::SerialChainException & error) {
        EXPECT_EQ(error.code(), rbk::SerialChainError::invalid_policy);
      }
    };

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  // The series threshold must be finite and strictly between zero and pi.
  for (const double invalid_threshold :
       {0.0, -1.0, std::numbers::pi, nan, infinity}) {
    rbk::NumericalPolicy policy;
    policy.series_angle_threshold = invalid_threshold;
    expect_invalid_policy(policy);
  }

  // The maximum angle must be finite and at least pi.
  const double just_below_pi = std::nextafter(std::numbers::pi, 0.0);

  for (const double invalid_maximum : {just_below_pi, nan, infinity}) {
    rbk::NumericalPolicy policy;
    policy.maximum_exponential_angle = invalid_maximum;
    expect_invalid_policy(policy);
  }
}

// Purpose: Distinguish the numerical angular bound from physical joint limits.
// For unit s, ||s q|| = |q|: +/-pi is accepted by a pi-rad policy, but
// immediately larger magnitudes fail even though q remains in [-4, 4] rad.
TEST(ForwardKinematicsTest, RespectsRevoluteExponentialAngleLimit)
{
  namespace rbk = rigid_body_kinematics;

  // Physical limits deliberately extend beyond the numerical angle limit.
  const rbk::RevoluteLimits limits =
    rbk::RevoluteLimits::from_bounds(-4.0, 4.0);

  const rbk::RevoluteJoint joint = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {joint});

  rbk::NumericalPolicy policy;
  policy.maximum_exponential_angle = std::numbers::pi;

  const double just_above_maximum =
    std::nextafter(policy.maximum_exponential_angle, 4.0);

  Eigen::VectorXd joint_coordinates(1);

  for (const double sign : {-1.0, 1.0}) {
    // The numerical boundary is inclusive.
    joint_coordinates(0) = sign * policy.maximum_exponential_angle;

    EXPECT_NO_THROW(static_cast<void>(
      rbk::space_form_forward_kinematics(model, joint_coordinates, policy)))
      << "Boundary coordinate: " << joint_coordinates(0);

    // Still inside the physical interval, but numerically unsupported.
    joint_coordinates(0) = sign * just_above_maximum;

    try {
      static_cast<void>(
        rbk::space_form_forward_kinematics(model, joint_coordinates, policy));

      FAIL() << "Expected unsupported_magnitude for coordinate "
             << joint_coordinates(0);
    } catch (const rbk::SerialChainException & error) {
      EXPECT_EQ(error.code(), rbk::SerialChainError::unsupported_magnitude)
        << "Coordinate: " << joint_coordinates(0);
    }
  }
}

// Purpose: Prevent applying an angular bound to a displacement in metres.
// A prismatic screw has zero angular block, so a permitted q = 4 m succeeds
// under a pi-rad angular policy and gives p = (4, 0, 0) m with R = I_3.
TEST(
  ForwardKinematicsTest, AngularPolicyLimitDoesNotRestrictPrismaticDisplacement)
{
  namespace rbk = rigid_body_kinematics;

  const rbk::PrismaticLimits limits =
    rbk::PrismaticLimits::from_bounds(-5.0, 5.0);

  const rbk::PrismaticJoint joint =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(1.0, 0.0, 0.0), limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {joint});

  rbk::NumericalPolicy policy;
  policy.maximum_exponential_angle = std::numbers::pi;

  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << 4.0;

  const rbk::Transform3 actual =
    rbk::space_form_forward_kinematics(model, joint_coordinates, policy);

  // Independent expected result: 4 m along +x, with no rotation.
  Eigen::Matrix4d expected_matrix = Eigen::Matrix4d::Identity();
  expected_matrix(0, 3) = 4.0;

  const rbk::Transform3 expected =
    rbk::Transform3::from_matrix(expected_matrix);

  expect_pose_near(
    expected, actual, kTranslationToleranceMetres, kRotationToleranceRadians);
}

// Purpose: Distinguish arithmetic overflow from non-finite supplied input.
// Both q and the home x-position equal the largest finite double D_max.
// The final E_1 M requires x = D_max + D_max, which is not representable;
// the evaluator must throw unsupported_magnitude rather than return a pose.
TEST(ForwardKinematicsTest, RejectsOverflowDuringHomePoseComposition)
{
  namespace rbk = rigid_body_kinematics;

  const double largest_finite = std::numeric_limits<double>::max();

  // Allow any finite nonnegative prismatic coordinate.
  const rbk::PrismaticLimits limits =
    rbk::PrismaticLimits::from_bounds(0.0, std::nullopt);

  const rbk::PrismaticJoint joint =
    rbk::PrismaticJoint::from_axis(Eigen::Vector3d(1.0, 0.0, 0.0), limits);

  // The home translation is large but finite.
  Eigen::Matrix4d home_matrix = Eigen::Matrix4d::Identity();
  home_matrix(0, 3) = largest_finite;

  const rbk::Transform3 home_pose = rbk::Transform3::from_matrix(home_matrix);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(home_pose, {joint});

  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << largest_finite;

  try {
    static_cast<void>(
      rbk::space_form_forward_kinematics(model, joint_coordinates));

    FAIL() << "Expected unsupported_magnitude when the final "
              "translation overflows.";
  } catch (const rbk::SerialChainException & error) {
    EXPECT_EQ(error.code(), rbk::SerialChainError::unsupported_magnitude);
  }
}

// Purpose: reject overflow in exponential coordinates eta = S * q.
// The screw and coordinate are individually finite, but scaling the linear
// component -0.75 D_max by q = 2 produces an unrepresentable -1.5 D_max.
TEST(ForwardKinematicsTest, RejectsOverflowDuringScrewScaling)
{
  namespace rbk = rigid_body_kinematics;

  const double largest_finite = std::numeric_limits<double>::max();
  const double axis_offset_metres = 0.75 * largest_finite;

  // The requested angle is within both the physical and numerical limits.
  const rbk::RevoluteLimits limits =
    rbk::RevoluteLimits::from_bounds(-std::numbers::pi, std::numbers::pi);

  const rbk::RevoluteJoint joint = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(axis_offset_metres, 0.0, 0.0), limits);

  const rbk::SerialChainModel model =
    rbk::SerialChainModel::from_home_and_joints(
      rbk::Transform3::identity(), {joint});

  Eigen::VectorXd joint_coordinates(1);
  joint_coordinates << 2.0;

  try {
    static_cast<void>(
      rbk::space_form_forward_kinematics(model, joint_coordinates));

    FAIL() << "Expected unsupported_magnitude when screw scaling overflows.";
  } catch (const rbk::SerialChainException & error) {
    EXPECT_EQ(error.code(), rbk::SerialChainError::unsupported_magnitude);
  }
}
