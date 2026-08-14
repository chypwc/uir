#include "rigid_body_kinematics/lie_algebra.hpp"

#include <cmath>

#include "rigid_body_kinematics/geometry_error.hpp"

namespace rigid_body_kinematics
{
Eigen::Matrix3d hat_so3(const Eigen::Vector3d & vector)
{
  if (!vector.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Hat-map input components must be finite.");
  }

  Eigen::Matrix3d skew_matrix;

  // clang-format off
  skew_matrix <<
    0.0,        -vector.z(), vector.y(),
    vector.z(),  0.0,       -vector.x(),
    -vector.y(), vector.x(), 0.0;
  // clang-format on

  return skew_matrix;
}

Eigen::Vector3d vee_so3(
  const Eigen::Matrix3d & matrix, const NumericalPolicy & policy)
{
  const bool policy_is_valid =
    std::isfinite(policy.tangent_tolerance) && policy.tangent_tolerance >= 0.0;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Tangent tolerance must be finite and non-negative.");
  }
  if (!matrix.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Vee-map matrix entries must be finite.");
  }

  // multiply 1/2 first to reduce the overflow risk
  const Eigen::Matrix3d half_matrix = 0.5 * matrix;

  const Eigen::Matrix3d symmetric_part = half_matrix + half_matrix.transpose();

  if (!symmetric_part.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Symmetric-part calculation produced a non-finite result.");
  }

  const double skew_residual = symmetric_part.cwiseAbs().maxCoeff();

  if (skew_residual > policy.tangent_tolerance) {
    throw GeometryException(
      GeometryError::invalid_tangent,
      "Matrix is not skew-symmetric within tolerance.");
  }

  const Eigen::Matrix3d skew_part = half_matrix - half_matrix.transpose();

  if (!skew_part.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Skew-part calculation produced a non-finite result.");
  }

  const Eigen::Vector3d vector(
    skew_part(2, 1), skew_part(0, 2), skew_part(1, 0));

  if (!vector.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Vee-map output is outside the finite numerical range.");
  }

  return vector;
}

// Convert a finite linear-first column into an se(3) tangent matrix.
Eigen::Matrix4d hat_se3(const Vector6LinearFirst & coordinates)
{
  if (!coordinates.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite,
      "SE(3) hat-map input components must be finite.");
  }

  const Eigen::Vector3d linear_part = coordinates.head<3>();
  const Eigen::Vector3d angular_part = coordinates.tail<3>();

  Eigen::Matrix4d tangent_matrix = Eigen::Matrix4d::Zero();

  tangent_matrix.block<3, 3>(0, 0) = hat_so3(angular_part);

  tangent_matrix.block<3, 1>(0, 3) = linear_part;

  return tangent_matrix;
}

Vector6LinearFirst vee_se3(
  const Eigen::Matrix4d & matrix, const NumericalPolicy & policy)
{
  const bool policy_is_valid =
    std::isfinite(policy.tangent_tolerance) && policy.tangent_tolerance >= 0.0;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Tangent tolerance must be finite and non-negative.");
  }

  if (!matrix.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite,
      "SE(3) vee-map matrix entries must be finite.");
  }

  const double bottom_row_residual = matrix.row(3).cwiseAbs().maxCoeff();
  if (bottom_row_residual > policy.tangent_tolerance) {
    throw GeometryException(
      GeometryError::invalid_tangent,
      "SE(3) tangent matrix must have a zero bottom row.");
  }

  const Eigen::Matrix3d angular_matrix = matrix.block<3, 3>(0, 0);

  const Eigen::Vector3d angular_part = vee_so3(angular_matrix, policy);

  const Eigen::Vector3d linear_part = matrix.block<3, 1>(0, 3);

  Vector6LinearFirst coordinates;
  coordinates.head<3>() = linear_part;
  coordinates.tail<3>() = angular_part;

  return coordinates;
}
}  // namespace rigid_body_kinematics
