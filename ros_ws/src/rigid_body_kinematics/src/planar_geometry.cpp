#include "rigid_body_kinematics/planar_geometry.hpp"

#include <cmath>

#include "rigid_body_kinematics/geometry_error.hpp"

namespace rigid_body_kinematics
{
Transform3 embed_planar_pose(
  const PlanarPose & pose, const NumericalPolicy & policy)
{
  constexpr double kPi = 3.141592653589793238462643383279502884;

  const bool yaw_policy_is_valid =
    std::isfinite(policy.maximum_exponential_angle) &&
    policy.maximum_exponential_angle >= kPi;

  if (!yaw_policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Maximum exponential angle must be finite and at least pi.");
  }

  const bool pose_is_finite = std::isfinite(pose.x_metres) &&
                              std::isfinite(pose.y_metres) &&
                              std::isfinite(pose.yaw_radians);

  if (!pose_is_finite) {
    throw GeometryException(
      GeometryError::non_finite, "Planar-pose components must be finite.");
  }

  if (std::abs(pose.yaw_radians) > policy.maximum_exponential_angle) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Planar yaw exceeds the supported maximum angle.");
  }

  const double cos_theta = std::cos(pose.yaw_radians);
  const double sin_theta = std::sin(pose.yaw_radians);

  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<
    cos_theta, -sin_theta, 0.0, pose.x_metres,
    sin_theta,  cos_theta, 0.0, pose.y_metres,
    0.0,        0.0,       1.0, 0.0,
    0.0,        0.0,       0.0, 1.0;
  // clang-format on

  return Transform3::from_matrix(matrix, policy);
}
}  // namespace rigid_body_kinematics
