#include "rigid_body_kinematics/rotation3.hpp"

#include <Eigen/LU>  // matrix.determinant()
#include <cmath>     // std::isfinite(), std::abs()

#include "rigid_body_kinematics/geometry_error.hpp"

namespace rigid_body_kinematics
{
Rotation3::Rotation3(const Eigen::Matrix3d & matrix) : matrix_(matrix) {}

Rotation3 Rotation3::identity()
{
  return Rotation3(Eigen::Matrix3d::Identity());
}

Rotation3 Rotation3::from_matrix(
  const Eigen::Matrix3d & matrix, const NumericalPolicy & policy)
{
  const bool policy_is_valid = std::isfinite(policy.orthogonality_tolerance) &&
                               std::isfinite(policy.determinant_tolerance) &&
                               policy.orthogonality_tolerance >= 0.0 &&
                               policy.determinant_tolerance >= 0.0;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Rotation tolerance must be finite and non-negative.");
  }

  if (!matrix.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Rotation matrix entries must be finite.");
  }

  const Eigen::Matrix3d orthogonality_error =
    matrix.transpose() * matrix - Eigen::Matrix3d::Identity();

  const double orthogonality_residual =
    orthogonality_error.reshaped().stableNorm();  // Frobenius norm

  const double determinant = matrix.determinant();

  if (!std::isfinite(orthogonality_residual) || !std::isfinite(determinant)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation validation produced a non-finite result.");
  }

  const double determinant_error = std::abs(determinant - 1.0);

  if (
    orthogonality_residual > policy.orthogonality_tolerance ||
    determinant_error > policy.determinant_tolerance) {
    throw GeometryException(
      GeometryError::invalid_rotation,
      "Matrix does not satisfy the SO(3) conditions.");
  }

  return Rotation3(matrix);
}

Eigen::Vector3d Rotation3::rotate_vector(const Eigen::Vector3d & vector_b) const
{
  if (!vector_b.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Free-vector components must be finite.");
  }

  const Eigen::Vector3d vector_a = matrix_ * vector_b;

  if (!vector_a.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotated free vector is outside the finite numerical range.");
  }
  return vector_a;
}

}  // namespace rigid_body_kinematics
