#include <Eigen/Core>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <numbers>

#include "rigid_body_kinematics/forward_kinematics.hpp"
#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/serial_chain_model.hpp"
#include "rigid_body_kinematics/transform3.hpp"

int main()
{
  namespace rbk = rigid_body_kinematics;

  try {
    const double first_link_length_metres = 2.0;
    const double second_link_length_metres = 1.0;

    const rbk::RevoluteLimits limits =
      rbk::RevoluteLimits::from_bounds(-std::numbers::pi, std::numbers::pi);

    // Both axis descriptions refer to the home configuration.
    const rbk::RevoluteJoint joint_1 = rbk::RevoluteJoint::from_axis(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), limits);

    const rbk::RevoluteJoint joint_2 = rbk::RevoluteJoint::from_axis(
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d(first_link_length_metres, 0.0, 0.0), limits);

    // Each endpoint has its own home pose.
    Eigen::Matrix4d elbow_home_matrix = Eigen::Matrix4d::Identity();
    elbow_home_matrix(0, 3) = first_link_length_metres;

    Eigen::Matrix4d end_home_matrix = Eigen::Matrix4d::Identity();
    end_home_matrix(0, 3) =
      first_link_length_metres + second_link_length_metres;

    const rbk::SerialChainModel elbow_model =
      rbk::SerialChainModel::from_home_and_joints(
        rbk::Transform3::from_matrix(elbow_home_matrix), {joint_1});

    const rbk::SerialChainModel end_model =
      rbk::SerialChainModel::from_home_and_joints(
        rbk::Transform3::from_matrix(end_home_matrix), {joint_1, joint_2});

    Eigen::VectorXd joint_coordinates(2);
    joint_coordinates << std::numbers::pi / 2.0, -std::numbers::pi / 2.0;

    Eigen::VectorXd elbow_coordinates(1);
    elbow_coordinates(0) = joint_coordinates(0);

    const rbk::Transform3 elbow_pose =
      rbk::space_form_forward_kinematics(elbow_model, elbow_coordinates);

    const rbk::Transform3 end_pose =
      rbk::space_form_forward_kinematics(end_model, joint_coordinates);

    const Eigen::Vector3d elbow_position_metres =
      elbow_pose.transform_point(Eigen::Vector3d::Zero());

    std::cout << "Elbow position in world [m]: "
              << elbow_position_metres.transpose() << '\n';
    std::cout << "End-effector transform T_se:\n" << end_pose.matrix() << '\n';

  } catch (const std::exception & error) {
    std::cerr << "Spatial 2R evaluation failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
