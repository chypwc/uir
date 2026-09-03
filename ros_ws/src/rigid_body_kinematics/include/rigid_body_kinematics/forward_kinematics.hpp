#ifndef RIGID_BODY_KINEMATICS_FORWARD_KINEMATICS_HPP_
#define RIGID_BODY_KINEMATICS_FORWARD_KINEMATICS_HPP_

#include <Eigen/Core>

#include "rigid_body_kinematics/numerical_policy.hpp"
#include "rigid_body_kinematics/serial_chain_model.hpp"
#include "rigid_body_kinematics/transform3.hpp"

namespace rigid_body_kinematics
{

[[nodiscard]] Transform3 space_form_forward_kinematics(
  const SerialChainModel & model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy & policy = NumericalPolicy{});

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_FORWARD_KINEMATICS_HPP_
