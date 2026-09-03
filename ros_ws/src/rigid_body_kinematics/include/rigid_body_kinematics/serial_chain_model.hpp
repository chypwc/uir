#ifndef RIGID_BODY_KINEMATICS_SERIAL_CHAIN_MODEL_HPP_
#define RIGID_BODY_KINEMATICS_SERIAL_CHAIN_MODEL_HPP_

#include <cstddef>
#include <vector>

#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/transform3.hpp"

namespace rigid_body_kinematics
{
class SerialChainModel
{
public:
  [[nodiscard]] static SerialChainModel from_home_and_joints(
    Transform3 home_pose, std::vector<JointDefinition> joint_definitions);

  [[nodiscard]] const Transform3 & home_pose() const noexcept;

  [[nodiscard]] const std::vector<JointDefinition> & joint_definitions()
    const noexcept;

  [[nodiscard]] std::size_t joint_count() const noexcept;

private:
  SerialChainModel(
    Transform3 home_pose, std::vector<JointDefinition> joint_definitions);

  Transform3 home_pose_;
  std::vector<JointDefinition> joint_definitions_;
};

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_SERIAL_CHAIN_MODEL_HPP_
