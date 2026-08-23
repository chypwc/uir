#ifndef RIGID_BODY_KINEMATICS__PLANAR_GEOMETRY_HPP_
#define RIGID_BODY_KINEMATICS__PLANAR_GEOMETRY_HPP_

#include "rigid_body_kinematics/numerical_policy.hpp"
#include "rigid_body_kinematics/transform3.hpp"

namespace rigid_body_kinematics
{
struct PlanarPose
{
  double x_metres;
  double y_metres;
  double yaw_radians;
};

[[nodiscard]] Transform3 embed_planar_pose(
  const PlanarPose & pose, const NumericalPolicy & policy = NumericalPolicy{});
}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS__PLANAR_GEOMETRY_HPP_
