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

PlanarPose extract_planar_pose(
  const Transform3 & transform, const NumericalPolicy & policy)
{
  const bool policy_is_valid =
    std::isfinite(policy.planar_orientation_tolerance) &&
    policy.planar_orientation_tolerance >= 0.0 &&
    std::isfinite(policy.planar_position_tolerance) &&
    policy.planar_position_tolerance >= 0.0;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "planar_orientation_tolerance and planar_position_tolerance should be "
      "finite and non-negative");
  }

  const Eigen::Matrix4d transform_matrix = transform.matrix();

  const Eigen::Vector3d e_z(0.0, 0.0, 1.0);
  const Eigen::Matrix3d rotation = transform_matrix.block<3, 3>(0, 0);
  const double planar_orientation_error = (rotation * e_z - e_z).stableNorm();

  if (planar_orientation_error > policy.planar_orientation_tolerance) {
    throw GeometryException(
      GeometryError::non_planar,
      "Planar orientation does not preserve the positive z-axis.");
  }

  if (std::abs(transform_matrix(2, 3)) > policy.planar_position_tolerance) {
    throw GeometryException(
      GeometryError::non_planar,
      "Vertical translation exceeds the planar-position tolerance.");
  }

  constexpr double kPi = 3.141592653589793238462643383279502884;

  double yaw_radians =
    std::atan2(transform_matrix(1, 0), transform_matrix(0, 0));

  // Ensure the yaw_radians is in (-pi, pi].
  if (yaw_radians <= -kPi) {
    yaw_radians = kPi;
  }

  return PlanarPose{
    transform_matrix(0, 3), transform_matrix(1, 3), yaw_radians};
}

QuaternionXYZW planar_yaw_to_quaternion(
  double yaw_radians, const NumericalPolicy & policy)
{
  constexpr double kPi = 3.141592653589793238462643383279502884;

  // 1. Validate the policy.
  const bool policy_is_valid =
    std::isfinite(policy.maximum_exponential_angle) &&
    policy.maximum_exponential_angle >= kPi;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Maximum exponential angle must be finite and at least pi.");
  }

  // 2. Validate the input.
  if (!std::isfinite(yaw_radians)) {
    throw GeometryException(
      GeometryError::non_finite, "Planar yaw must be finite.");
  }

  if (std::abs(yaw_radians) > policy.maximum_exponential_angle) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Planar yaw exceeds the supported maximum angle.");
  }

  // 3. Calculate the raw quaternion.
  double z = std::sin(0.5 * yaw_radians);
  double w = std::cos(0.5 * yaw_radians);

  // 4. Normalize explicitly.
  const double norm = std::hypot(z, w);  // Use hypotenuse to compute the norm.

  if (!std::isfinite(norm) || norm == 0.0) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Planar-yaw quaternion norm is outside the supported range.");
  }

  z /= norm;
  w /= norm;

  // rotation axis is the z-axis.
  return QuaternionXYZW{0.0, 0.0, z, w};
}

}  // namespace rigid_body_kinematics
