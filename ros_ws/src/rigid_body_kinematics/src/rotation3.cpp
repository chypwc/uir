#include "rigid_body_kinematics/rotation3.hpp"

#include <Eigen/LU>  // matrix.determinant()
#include <cmath>     // std::isfinite(), std::abs()
#include <numbers>

#include "rigid_body_kinematics/geometry_error.hpp"
#include "rigid_body_kinematics/lie_algebra.hpp"

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

Rotation3 Rotation3::from_rotation_vector(
  const Eigen::Vector3d & rotation_vector, const NumericalPolicy & policy)
{
  const bool policy_is_valid =
    std::isfinite(policy.series_angle_threshold) &&
    std::isfinite(policy.maximum_exponential_angle) &&
    policy.series_angle_threshold > 0.0 &&
    policy.series_angle_threshold < std::numbers::pi &&
    policy.maximum_exponential_angle >= std::numbers::pi;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Exponential thresholds do not satisfy the required bounds.");
  }

  if (!rotation_vector.allFinite()) {
    throw GeometryException(
      GeometryError::non_finite, "Rotation-vector components must be finite.");
  }

  const double theta = rotation_vector.stableNorm();

  if (!std::isfinite(theta)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation-vector norm is outside the finite numerical range.");
  }

  if (theta > policy.maximum_exponential_angle) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation-vector angle exceeds the supported maximum.");
  }

  if (theta == 0.0) {
    return identity();
  }

  const double theta_squared = theta * theta;

  double coefficient_a;
  double coefficient_b;

  if (theta <= policy.series_angle_threshold) {
    const double theta_fourth = theta_squared * theta_squared;
    const double theta_sixth = theta_fourth * theta_squared;

    coefficient_a =
      1.0 - theta_squared / 6.0 + theta_fourth / 120.0 - theta_sixth / 5040.0;

    coefficient_b =
      0.5 - theta_squared / 24.0 + theta_fourth / 720.0 - theta_sixth / 40320.0;
  } else {
    coefficient_a = std::sin(theta) / theta;
    coefficient_b = (1.0 - std::cos(theta)) / theta_squared;
  }

  if (!std::isfinite(coefficient_a) || !std::isfinite(coefficient_b)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rodrigues coefficients are not finite.");
  }

  const Eigen::Matrix3d rotation_generator = hat_so3(rotation_vector);

  const Eigen::Matrix3d rotation_matrix =
    Eigen::Matrix3d::Identity() + coefficient_a * rotation_generator +
    coefficient_b * rotation_generator * rotation_generator;

  if (!rotation_matrix.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation exponential produced a non-finite matrix.");
  }

  return Rotation3(rotation_matrix);
}

RotationLogResult Rotation3::to_principal_rotation_vector(
  const NumericalPolicy & policy) const
{
  const double near_pi_boundary = std::numbers::pi - policy.near_pi_tolerance;

  /*
  theta = 0                                             identity
  0 < theta <= series_angle_threshold                   small_angle
  series_angle_threshold < theta <
    pi - near_pi_tolerance                             nominal
  pi - near_pi_tolerance <= theta <= pi               near_pi
  */
  const bool policy_is_valid = std::isfinite(policy.orthogonality_tolerance) &&
                               std::isfinite(policy.series_angle_threshold) &&
                               std::isfinite(policy.near_pi_tolerance) &&
                               policy.orthogonality_tolerance >= 0.0 &&
                               policy.series_angle_threshold > 0.0 &&
                               policy.near_pi_tolerance >= 0.0 &&
                               policy.near_pi_tolerance < std::numbers::pi &&
                               policy.series_angle_threshold < near_pi_boundary;

  if (!policy_is_valid) {
    throw GeometryException(
      GeometryError::invalid_policy,
      "Logarithm thresholds do not satisfy the required bounds.");
  }

  // c_theta = (trace(R) - 1) / 2 = cos(theta).
  double cosine_theta = 0.5 * (matrix_.trace() - 1.0);

  if (!std::isfinite(cosine_theta)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Principal-angle cosine is not finite.");
  }

  if (cosine_theta > 1.0) {
    const double overshoot = cosine_theta - 1.0;

    // Reuse the dimensionless orthogonality tolerance for cosine clamping.
    if (overshoot > policy.orthogonality_tolerance) {
      throw GeometryException(
        GeometryError::invalid_rotation,
        "Principal-angle cosine exceeds its valid range.");
    }

    cosine_theta = 1.0;
  } else if (cosine_theta < -1.0) {
    const double overshoot = -1.0 - cosine_theta;

    if (overshoot > policy.orthogonality_tolerance) {
      throw GeometryException(
        GeometryError::invalid_rotation,
        "Principal-angle cosine exceeds its valid range.");
    }

    cosine_theta = -1.0;
  }

  // u = vee(R - R^T) = 2 sin(theta) s_ω.
  // clang-format off
  const Eigen::Vector3d skew_difference(
    matrix_(2, 1) - matrix_(1, 2),
    matrix_(0, 2) - matrix_(2, 0),
    matrix_(1, 0) - matrix_(0, 1));
  // clang-format on

  // sin(theta) = ||u|| / 2; it is non-negative on the principal branch.
  const double sine_theta = 0.5 * skew_difference.stableNorm();

  if (!std::isfinite(sine_theta)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Principal-angle sine is not finite.");
  }

  // theta = atan2(sin(theta), cos(theta)) lies in [0, pi].
  const double theta = std::atan2(sine_theta, cosine_theta);

  if (!std::isfinite(theta)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Principal rotation angle is not finite.");
  }

  if (theta == 0.0) {
    // The identity has no unique axis, so return zero before normalizing one.
    return RotationLogResult{
      Eigen::Vector3d::Zero(), RotationLogBranch::identity};
  }

  if (theta >= near_pi_boundary) {
    // Near pi, u approaches zero. Recover Q = s s^T from the symmetric part,
    // where s = s_ω is the unit rotation axis.
    const double denominator = 1.0 - cosine_theta;

    const Eigen::Matrix3d symmetric_part =
      0.5 * (matrix_ + matrix_.transpose());

    // Q = [ (R + R^T) / 2 - cos(theta) I ] / (1 - cos(theta))
    const Eigen::Matrix3d axis_outer_product =
      (symmetric_part - cosine_theta * Eigen::Matrix3d::Identity()) /
      denominator;

    if (!axis_outer_product.allFinite()) {
      throw GeometryException(
        GeometryError::unsupported_magnitude,
        "Near-pi axis calculation produced a non-finite matrix.");
    }

    // Select the largest diagonal entry. A strict comparison keeps
    // the lowest index when two entries are equal.
    Eigen::Index dominant_index = 0;

    // k = argmax_i s_{ω, i}^2
    for (Eigen::Index index = 1; index < 3; ++index) {
      if (
        axis_outer_product(index, index) >
        axis_outer_product(dominant_index, dominant_index)) {
        dominant_index = index;
      }
    }

    // s_{ω, k}^2
    double dominant_component_squared =
      axis_outer_product(dominant_index, dominant_index);

    if (dominant_component_squared < 0.0) {
      const double undershoot = -dominant_component_squared;
      if (undershoot > policy.orthogonality_tolerance) {
        throw GeometryException(
          GeometryError::invalid_rotation,
          "Near-pi axis square is materially negative.");
      }

      dominant_component_squared = 0.0;
    }

    // |s_{ω, k}|
    const double dominant_component = std::sqrt(dominant_component_squared);

    if (!std::isfinite(dominant_component) || dominant_component == 0.0) {
      throw GeometryException(
        GeometryError::invalid_rotation,
        "Near-pi axis has no recoverable dominant component.");
    }

    // Column k of Q is s_ω * s_k, so divide it by the recovered s_k.
    Eigen::Vector3d axis =
      axis_outer_product.col(dominant_index) / dominant_component;

    // Preserve the selected positive square root explicitly.
    // Avoid accumulating another floating-point division error.
    axis(dominant_index) = dominant_component;

    if (!axis.allFinite()) {
      throw GeometryException(
        GeometryError::unsupported_magnitude,
        "Near-pi axis components are not finite.");
    }

    const double axis_norm = axis.stableNorm();

    if (!std::isfinite(axis_norm)) {
      throw GeometryException(
        GeometryError::unsupported_magnitude,
        "Near-pi axis norm is not finite.");
    }

    if (axis_norm == 0.0) {
      throw GeometryException(
        GeometryError::invalid_rotation, "Near-pi axis has zero norm.");
    }

    // Restore unit length after the floating-point component recovery.
    axis /= axis_norm;

    // u = 2 sin(theta) s_ω --> s_ω^T  u  = 2 sin(theta)
    const double skew_alignment = axis.dot(skew_difference);

    if (!std::isfinite(skew_alignment)) {
      throw GeometryException(
        GeometryError::unsupported_magnitude,
        "Near-pi axis sign calculation is not finite.");
    }

    // Below pi, align the recovered axis sign with u.
    // At exact pi, keep the selected dominant component positive.
    if (skew_alignment < 0.0) {
      axis = -axis;
    }

    // phi = theta s_ω.
    const Eigen::Vector3d rotation_vector = theta * axis;

    if (!rotation_vector.allFinite()) {
      throw GeometryException(
        GeometryError::unsupported_magnitude,
        "Near-pi rotation vector is not finite.");
    }

    return RotationLogResult{rotation_vector, RotationLogBranch::near_pi};
  }  // near-pi branch

  double recovery_factor;
  RotationLogBranch branch;

  if (theta <= policy.series_angle_threshold) {
    // theta / (2 sin(theta)) ~=
    // 1/2 + theta^2/12 + 7 theta^4/720 + O(theta^6).
    const double theta_squared = theta * theta;
    const double theta_fourth = theta_squared * theta_squared;

    recovery_factor = 0.5 + theta_squared / 12.0 + 7.0 * theta_fourth / 720.0;

    branch = RotationLogBranch::small_angle;
  } else {
    if (sine_theta == 0.0) {
      throw GeometryException(
        GeometryError::invalid_rotation,
        "Nominal logarithm has zero axis magnitude.");
    }

    // Away from zero and pi: phi = theta s_ω = theta u / (2 sin(theta)).
    recovery_factor = theta / (2.0 * sine_theta);

    branch = RotationLogBranch::nominal;
  }

  if (!std::isfinite(recovery_factor)) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation-vector recovery factor is not finite.");
  }

  // Both branches use phi = recovery_factor * u.
  const Eigen::Vector3d rotation_vector = recovery_factor * skew_difference;

  if (!rotation_vector.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Principal rotation vector is not finite.");
  }

  return RotationLogResult{rotation_vector, branch};
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

Eigen::Matrix3d Rotation3::matrix() const { return matrix_; }

Rotation3 Rotation3::compose(const Rotation3 & rotation_bc) const
{
  const Eigen::Matrix3d matrix_ac = matrix_ * rotation_bc.matrix_;

  if (!matrix_ac.allFinite()) {
    throw GeometryException(
      GeometryError::unsupported_magnitude,
      "Rotation composition produced a non-finite result.");
  }

  return Rotation3(matrix_ac);
}

Rotation3 Rotation3::inverse() const
{
  const Eigen::Matrix3d matrix_ba = matrix_.transpose();
  return Rotation3(matrix_ba);
}
}  // namespace rigid_body_kinematics
