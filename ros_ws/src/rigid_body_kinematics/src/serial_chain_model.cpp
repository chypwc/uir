#include "rigid_body_kinematics/serial_chain_model.hpp"

#include <utility>
#include <vector>

#include "rigid_body_kinematics/serial_chain_error.hpp"

namespace rigid_body_kinematics
{

SerialChainModel SerialChainModel::from_home_and_joints(
  Transform3 home_pose, std::vector<JointDefinition> joint_definitions)
{
  if (joint_definitions.empty()) {
    throw SerialChainException(
      SerialChainError::invalid_model,
      "A serial-chain model must contain at least one joint.");
  }

  return SerialChainModel(std::move(home_pose), std::move(joint_definitions));
}

SerialChainModel::SerialChainModel(
  Transform3 home_pose, std::vector<JointDefinition> joint_definitions)
: home_pose_(std::move(home_pose)),
  joint_definitions_(std::move(joint_definitions))
{
}

const Transform3 & SerialChainModel::home_pose() const noexcept
{
  return home_pose_;
}

const std::vector<JointDefinition> & SerialChainModel::joint_definitions()
  const noexcept
{
  return joint_definitions_;
}

std::size_t SerialChainModel::joint_count() const noexcept
{
  return joint_definitions_.size();
}

}  // namespace rigid_body_kinematics
