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

// Unit quaternion fields in ROS 2 xyzw order.
struct QuaternionXYZW
{
  double x;
  double y;
  double z;
  double w;
};

[[nodiscard]] Transform3 embed_planar_pose(
  const PlanarPose & pose, const NumericalPolicy & policy = NumericalPolicy{});

// Extract [x, y, yaw angle] only from a validated planar transform.
// Translation is measured in metres and yaw is returned in (-pi, pi].
[[nodiscard]] PlanarPose extract_planar_pose(
  const Transform3 & transform,
  const NumericalPolicy & policy = NumericalPolicy{});

// Convert finite planar yaw in radians to a normalized quaternion
// in ROS 2 xyzw field order.
[[nodiscard]] QuaternionXYZW planar_yaw_to_quaternion(
  double yaw_radians, const NumericalPolicy & policy = NumericalPolicy{});

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS__PLANAR_GEOMETRY_HPP_
