#include "rigid_body_kinematics/joint_definition.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <utility>

#include "rigid_body_kinematics/serial_chain_error.hpp"

namespace rigid_body_kinematics
{
namespace
{
Eigen::Vector3d normalize_axis_direction(const Eigen::Vector3d & axis_direction)
{
  if (!axis_direction.allFinite()) {
    throw SerialChainException(
      SerialChainError::non_finite, "Joint axis direction must be finite.");
  }

  const double direction_norm = axis_direction.stableNorm();

  if (direction_norm == 0.0) {
    throw SerialChainException(
      SerialChainError::invalid_model, "Joint axis direction must be nonzero.");
  }

  if (!std::isfinite(direction_norm)) {
    throw SerialChainException(
      SerialChainError::unsupported_magnitude,
      "Joint axis direction cannot be normalized safely.");
  }

  const Eigen::Vector3d normalized_direction = axis_direction / direction_norm;

  if (!normalized_direction.allFinite()) {
    throw SerialChainException(
      SerialChainError::unsupported_magnitude,
      "Normalized joint axis direction is not finite.");
  }

  return normalized_direction;
}

}  // namespace

RevoluteJoint RevoluteJoint::from_axis(
  const Eigen::Vector3d & space_axis_direction,
  const Eigen::Vector3d & space_axis_point_metres, RevoluteLimits limits)
{
  if (!space_axis_point_metres.allFinite()) {
    throw SerialChainException(
      SerialChainError::non_finite, "Revolute axis point must be finite.");
  }

  const Eigen::Vector3d normalized_direction =
    normalize_axis_direction(space_axis_direction);

  const Eigen::Vector3d linear_screw_block =
    -normalized_direction.cross(space_axis_point_metres);

  if (!linear_screw_block.allFinite()) {
    throw SerialChainException(
      SerialChainError::unsupported_magnitude,
      "Revolute screw-axis calculation is not finite.");
  }

  Vector6LinearFirst space_screw_axis;
  space_screw_axis.head<3>() = linear_screw_block;
  space_screw_axis.tail<3>() = normalized_direction;

  return RevoluteJoint(
    normalized_direction, space_axis_point_metres, std::move(limits),
    space_screw_axis);
}

RevoluteJoint::RevoluteJoint(
  Eigen::Vector3d space_axis_direction, Eigen::Vector3d space_axis_point_metres,
  RevoluteLimits limits, Vector6LinearFirst space_screw_axis)
: space_axis_direction_(std::move(space_axis_direction)),
  space_axis_point_metres_(std::move(space_axis_point_metres)),
  limits_(std::move(limits)),
  space_screw_axis_(std::move(space_screw_axis))
{
}

const Eigen::Vector3d & RevoluteJoint::space_axis_direction() const noexcept
{
  return space_axis_direction_;
}

const Eigen::Vector3d & RevoluteJoint::space_axis_point_metres() const noexcept
{
  return space_axis_point_metres_;
}

const Vector6LinearFirst & RevoluteJoint::space_screw_axis() const noexcept
{
  return space_screw_axis_;
}

const RevoluteLimits & RevoluteJoint::limits() const noexcept
{
  return limits_;
}

}  // namespace rigid_body_kinematics
