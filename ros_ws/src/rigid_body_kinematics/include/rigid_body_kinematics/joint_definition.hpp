#ifndef RIGID_BODY_KINEMATICS_JOINT_DEFINITION_HPP_
#define RIGID_BODY_KINEMATICS_JOINT_DEFINITION_HPP_

#include <Eigen/Core>
#include <variant>

#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/lie_algebra.hpp"

namespace rigid_body_kinematics
{

class RevoluteJoint
{
public:
  [[nodiscard]] static RevoluteJoint from_axis(
    const Eigen::Vector3d & space_axis_direction,
    const Eigen::Vector3d & space_axis_point_metres, RevoluteLimits limits);

  [[nodiscard]] const Eigen::Vector3d & space_axis_direction() const noexcept;

  [[nodiscard]] const Eigen::Vector3d & space_axis_point_metres()
    const noexcept;

  [[nodiscard]] const Vector6LinearFirst & space_screw_axis() const noexcept;

  [[nodiscard]] const RevoluteLimits & limits() const noexcept;

private:
  RevoluteJoint(
    Eigen::Vector3d space_axis_direction,
    Eigen::Vector3d space_axis_point_metres, RevoluteLimits limits,
    Vector6LinearFirst space_screw_axis);

  Eigen::Vector3d space_axis_direction_;
  Eigen::Vector3d space_axis_point_metres_;
  RevoluteLimits limits_;
  Vector6LinearFirst space_screw_axis_;
};

class PrismaticJoint
{
public:
  [[nodiscard]] static PrismaticJoint from_axis(
    const Eigen::Vector3d & space_axis_direction, PrismaticLimits limits);

  [[nodiscard]] const Eigen::Vector3d & space_axis_direction() const noexcept;

  [[nodiscard]] const Vector6LinearFirst & space_screw_axis() const noexcept;

  [[nodiscard]] const PrismaticLimits & limits() const noexcept;

private:
  PrismaticJoint(
    Eigen::Vector3d space_axis_direction, PrismaticLimits limits,
    Vector6LinearFirst space_screw_axis);

  Eigen::Vector3d space_axis_direction_;
  PrismaticLimits limits_;
  Vector6LinearFirst space_screw_axis_;
};

using JointDefinition = std::variant<RevoluteJoint, PrismaticJoint>;

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_JOINT_DEFINITION_HPP_
