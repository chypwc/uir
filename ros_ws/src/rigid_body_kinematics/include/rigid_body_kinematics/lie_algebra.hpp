#ifndef RIGID_BODY_KINEMATICS_LIE_ALGEBRA_HPP_
#define RIGID_BODY_KINEMATICS_LIE_ALGEBRA_HPP_

#include <Eigen/Core>

#include "rigid_body_kinematics/numerical_policy.hpp"

namespace rigid_body_kinematics
{
// Convert a finite three-component column into its cross-product matrix
[[nodiscard]] Eigen::Matrix3d hat_so3(const Eigen::Vector3d & vector);

// Recover a three-component column from a validated skew matrix.
[[nodiscard]] Eigen::Vector3d vee_so3(
  const Eigen::Matrix3d & matrix,
  const NumericalPolicy & policy = NumericalPolicy{});

// Six-component column ordered as [linear; angular].
using Vector6LinearFirst = Eigen::Matrix<double, 6, 1>;

// Convert a finite linear-first column into an se(3) tangent matrix.
[[nodiscard]] Eigen::Matrix4d hat_se3(const Vector6LinearFirst & coordinates);

// Recover a linear-first column from a validated se(3) tangent matrix.
[[nodiscard]] Vector6LinearFirst vee_se3(
  const Eigen::Matrix4d & matrix,
  const NumericalPolicy & policy = NumericalPolicy{});

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_LIE_ALGEBRA_HPP_
