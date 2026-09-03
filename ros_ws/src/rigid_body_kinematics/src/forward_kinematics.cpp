#include "rigid_body_kinematics/forward_kinematics.hpp"

#include <cmath>
#include <cstddef>
#include <numbers>
#include <string>
#include <variant>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/serial_chain_error.hpp"

namespace rigid_body_kinematics
{
namespace
{

void validate_exponential_policy(const NumericalPolicy & policy)
{
  const bool policy_is_valid =
    std::isfinite(policy.series_angle_threshold) &&
    std::isfinite(policy.maximum_exponential_angle) &&
    policy.series_angle_threshold > 0.0 &&
    policy.series_angle_threshold < std::numbers::pi &&
    policy.maximum_exponential_angle >= std::numbers::pi;

  if (!policy_is_valid) {
    throw SerialChainException(
      SerialChainError::invalid_policy,
      "The exponential numerical policy is invalid.");
  }
}

bool joint_contains(
  const JointDefinition & joint_definition, double joint_coordinate)
{
  // Call the same coordinate-limit check using whichever joint type
  // the variant currently contains, and return the resulting bool.
  return std::visit(
    [joint_coordinate](const auto & joint) {
      return joint.limits().contains(joint_coordinate);
    },
    joint_definition);
}

const Vector6LinearFirst & space_screw_axis(
  const JointDefinition & joint_definition)
{
  return std::visit(
    [](const auto & joint) -> const Vector6LinearFirst & {
      return joint.space_screw_axis();
    },
    joint_definition);
}

}  // namespace

Transform3 space_form_forward_kinematics(
  const SerialChainModel & model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy & policy)
{
  validate_exponential_policy(policy);

  const Eigen::Index expected_coordinate_count =
    static_cast<Eigen::Index>(model.joint_count());

  if (joint_coordinates.size() != expected_coordinate_count) {
    throw SerialChainException(
      SerialChainError::dimension_mismatch,
      "The joint-coordinate count must equal the model joint count.");
  }

  // Check coordinates are finite.
  for (Eigen::Index index = 0; index < joint_coordinates.size(); ++index) {
    if (!std::isfinite(joint_coordinates(index))) {
      throw SerialChainException(
        SerialChainError::non_finite,
        "Joint " + std::to_string(index + 1) + " has a non-finite coordinate.");
    }
  }

  // Check coordinates are in joint limits.
  for (std::size_t index = 0; index < model.joint_count(); ++index) {
    const double coordinate =
      joint_coordinates(static_cast<Eigen::Index>(index));

    const JointDefinition & joint_definition =
      model.joint_definitions().at(index);

    if (!joint_contains(joint_definition, coordinate)) {
      throw SerialChainException(
        SerialChainError::joint_out_of_domain,
        "Joint " + std::to_string(index + 1) +
          " is outside its declared limits.");
    }

    if (
      std::holds_alternative<RevoluteJoint>(joint_definition) &&
      std::abs(coordinate) > policy.maximum_exponential_angle) {
      throw SerialChainException(
        SerialChainError::unsupported_magnitude,
        "Joint " + std::to_string(index + 1) +
          " exceeds the supported exponential angle.");
    }
  }

  Transform3 result = Transform3::identity();

  // I -> E_1 -> E_1 E_2 -> ... -> E_1 E_2 ... E_n
  for (std::size_t index = 0; index < model.joint_count(); ++index) {
    const double coordinate =
      joint_coordinates(static_cast<Eigen::Index>(index));

    const Vector6LinearFirst exponential_coordinates =
      space_screw_axis(model.joint_definitions().at(index)) * coordinate;

    if (!exponential_coordinates.allFinite()) {
      throw SerialChainException(
        SerialChainError::unsupported_magnitude,
        "Joint " + std::to_string(index + 1) +
          " produced non-finite exponential coordinates.");
    }

    try {
      const Transform3 joint_displacement =
        Transform3::from_exponential_coordinates(
          exponential_coordinates, policy);

      result = result.compose(joint_displacement);
    } catch (const GeometryException & error) {
      const SerialChainError serial_error =
        error.code() == GeometryError::invalid_policy
          ? SerialChainError::invalid_policy
          : SerialChainError::unsupported_magnitude;

      throw SerialChainException(
        serial_error, "Joint " + std::to_string(index + 1) +
                        " evaluation failed: " + error.what());
    }
  }  // for loop: forward kinematics

  // E_1 E_2 ... E_n M
  try {
    return result.compose(model.home_pose());
  } catch (const GeometryException & error) {
    throw SerialChainException(
      SerialChainError::unsupported_magnitude,
      std::string("Final home-pose composition failed: ") + error.what());
  }
}

}  // namespace rigid_body_kinematics
