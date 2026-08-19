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

Transform3 Transform3::from_exponential_coordinates(
  const Vector6LinearFirst & coordinates, const NumericalPolicy & policy)
{
  if (!coordinates.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite,
      "SE(3) exponential coordinates must be finite.");
  }

  // eta = [rho; phi], with rho in metres and phi in radians.
  const Eigen::Vector3d linear_coordinates = coordinates.head<3>();
  const Eigen::Vector3d rotation_vector = coordinates.tail<3>();

  // R = exp([phi]_x).  This also validates the angular policy and magnitude.
  const Rotation3 rotation =
    Rotation3::from_rotation_vector(rotation_vector, policy);

  const double theta = rotation_vector.stableNorm();

  if (!std::isfinite(theta)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation-vector norm is outside the finite numerical range.");
  }

  // When phi = 0, J(phi) = I and p = rho exactly.
  if (theta == 0.0) {
    return Transform3(rotation, linear_coordinates);
  }

  const double theta_squared = theta * theta;

  double coefficient_b;
  double coefficient_c;

  if (theta <= policy.series_angle_threshold) {
    const double theta_fourth = theta_squared * theta_squared;
    const double theta_sixth = theta_fourth * theta_squared;

    coefficient_b =
      0.5 - theta_squared / 24.0 + theta_fourth / 720.0 - theta_sixth / 40320.0;

    coefficient_c = 1.0 / 6.0 - theta_squared / 120.0 + theta_fourth / 5040.0 -
                    theta_sixth / 362880.0;
  } else {
    coefficient_b = (1.0 - std::cos(theta)) / theta_squared;
    coefficient_c = (theta - std::sin(theta)) / (theta_squared * theta);
  }

  if (!std::isfinite(coefficient_b) || !std::isfinite(coefficient_c)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "SE(3) left-Jaobian coefficients are not finite.");
  }

  // Phi = [phi]_x.
  const Eigen::Matrix3d rotation_generator = hat_so3(rotation_vector);

  // J(phi) = I + B(theta) Phi + C(theta) Phi^2.
  const Eigen::Matrix3d left_jacobian =
    Eigen::Matrix3d::Identity() + coefficient_b * rotation_generator +
    coefficient_c * rotation_generator * rotation_generator;

  if (!left_jacobian.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "SE(3) left Jacobian is outside the finite numerical range.");
  }

  // p = J(phi) rho.
  const Eigen::Vector3d translation = left_jacobian * linear_coordinates;

  if (!translation.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "SE(3) exponential translation is outside the finite numerical range.");
  }

  return Transform3(rotation, translation);
}

TransformLogResult Transform3::to_principal_exponential_coordinates(
  const NumericalPolicy & policy) const
{
  // Reuse policy validation in to_principal_rotation_vector().
  const RotationLogResult rotation_log_result =
    rotation_.to_principal_rotation_vector(policy);

  Vector6LinearFirst coordinates = Vector6LinearFirst::Zero();
  coordinates.tail<3>() = rotation_log_result.rotation_vector;

  // Identity rotation case.
  if (rotation_log_result.branch == RotationLogBranch::identity) {
    coordinates.head<3>() = translation_;

    const bool translation_is_zero = (translation_.array() == 0.0).all();

    const TransformLogBranch branch = translation_is_zero
                                        ? TransformLogBranch::identity
                                        : TransformLogBranch::pure_translation;
    return TransformLogResult{coordinates, branch};
  }

  TransformLogBranch branch;
  if (rotation_log_result.branch == RotationLogBranch::small_angle) {
    branch = TransformLogBranch::small_angle;
  } else if (rotation_log_result.branch == RotationLogBranch::near_pi) {
    branch = TransformLogBranch::near_pi;
  } else {
    branch = TransformLogBranch::nominal;
  }

  const Eigen::Vector3d phi = rotation_log_result.rotation_vector;
  const double theta = phi.stableNorm();

  if (!std::isfinite(theta)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Principal rotation-vector norm is not finite.");
  }

  const double theta_squared = theta * theta;

  double coefficient_d;
  if (theta <= policy.series_angle_threshold) {
    const double theta_fourth = theta_squared * theta_squared;
    const double theta_sixth = theta_fourth * theta_squared;

    coefficient_d = 1.0 / 12.0 + theta_squared / 720.0 +
                    theta_fourth / 30240.0 + theta_sixth / 1209600.0;
  } else {
    const double half_theta = 0.5 * theta;
    coefficient_d = 1.0 / theta_squared -
                    std::cos(half_theta) / (2.0 * theta * std::sin(half_theta));
  }

  if (!std::isfinite(coefficient_d)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Inverse left-Jacobian coefficient is not finite.");
  }

  const Eigen::Matrix3d rotation_vector_hat = hat_so3(phi);

  const Eigen::Matrix3d inverse_left_jacobian =
    Eigen::Matrix3d::Identity() - 0.5 * rotation_vector_hat +
    coefficient_d * rotation_vector_hat * rotation_vector_hat;

  if (!inverse_left_jacobian.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Inverse SE(3) left Jacobian is outside the finite numerical range.");
  }

  const Eigen::Vector3d linear_coordinates =
    inverse_left_jacobian * translation_;

  if (!linear_coordinates.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "SE(3) logarithm linear coordinates are outside the finite numerical "
      "range.");
  }

  coordinates.head<3>() = linear_coordinates;

  return TransformLogResult{coordinates, branch};
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
