#include "rigid_body_kinematics/transform3.hpp"

#include <cmath>

#include "rigid_body_kinematics/geometry_error.hpp"

namespace rigid_body_kinematics
{
Transform3::Transform3(
  const Rotation3 & rotation, const Eigen::Vector3d & translation)
: rotation_(rotation), translation_(translation)
{
}

Transform3 Transform3::identity()
{
  return Transform3(Rotation3::identity(), Eigen::Vector3d::Zero());
}

Transform3 Transform3::from_matrix(
  const Eigen::Matrix4d & matrix, const NumericalPolicy & policy)
{
  /*
  Input Eigen::Matrix4d
          ↓
  Validate NumericalPolicy
          ↓
  Check all 16 entries are finite
          ↓
  Check bottom row ≈ [0,0,0,1]
          ↓
  Extract upper-left 3×3 block
          ↓
  Rotation3::from_matrix()
          ↓
  Validate R ∈ SO(3)
          ↓
  Extract translation column
          ↓
  Transform3(rotation, translation)
  */
  const bool policy_is_valid =
    std::isfinite(policy.orthogonality_tolerance) &&
    std::isfinite(policy.determinant_tolerance) &&
    std::isfinite(policy.homogeneous_row_tolerance) &&
    policy.orthogonality_tolerance >= 0.0 &&
    policy.determinant_tolerance >= 0.0 &&
    policy.homogeneous_row_tolerance >= 0.0;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Transform tolerances must be finite and non-negative.");
  }

  if (!matrix.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Transform matrix entries must be finite.");
  }

  const Eigen::Vector4d expected_bottom_row(0.0, 0.0, 0.0, 1.0);

  const Eigen::Vector4d bottom_row_error =
    matrix.row(3).transpose() - expected_bottom_row;

  // infinity norm
  const double bottom_row_residual = bottom_row_error.cwiseAbs().maxCoeff();

  if (bottom_row_residual > policy.homogeneous_row_tolerance) {
    throw GeometryException(
      GeometryError::invalid_transform,
      "Transform must have homogeneous bottom row [0, 0, 0, 1].");
  }

  const Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);

  const Eigen::Vector3d translation = matrix.block<3, 1>(0, 3);

  const Rotation3 rotation = Rotation3::from_matrix(rotation_matrix, policy);

  return Transform3(rotation, translation);
}

Eigen::Vector3d Transform3::transform_point(
  const Eigen::Vector3d & point_b) const
{
  if (!point_b.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Point coordinates must be finite.");
  }

  const Eigen::Vector3d rotated_point = rotation_.rotate_vector(point_b);

  const Eigen::Vector3d point_a = rotated_point + translation_;

  if (!point_a.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Transformed point is outside the finite numerical range.");
  }
  return point_a;
}

// Reconstruct the homogeneous matrix with exact bottom row.
Eigen::Matrix4d Transform3::matrix() const
{
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

  result.block<3, 3>(0, 0) = rotation_.matrix();
  result.block<3, 1>(0, 3) = translation_;

  return result;
}

// Compose T_ab with T_bc to produce T_ac
Transform3 Transform3::compose(const Transform3 & transform_bc) const
{
  const Rotation3 rotation_ac = rotation_.compose(transform_bc.rotation_);
  const Eigen::Vector3d translated_position =
    rotation_.rotate_vector(transform_bc.translation_);
  const Eigen::Vector3d translation_ac = translated_position + translation_;

  if (!translation_ac.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Transform composition produced a non-finite translation.");
  }

  return Transform3(rotation_ac, translation_ac);
}

// Return the inverse transform T_ba
Transform3 Transform3::inverse() const
{
  const Rotation3 rotation_ba = rotation_.inverse();
  const Eigen::Vector3d translation_ba =
    rotation_ba.rotate_vector(-translation_);

  return Transform3(rotation_ba, translation_ba);
}
}  // namespace rigid_body_kinematics
