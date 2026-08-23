#ifndef RIGID_BODY_KINEMATICS_TRANSFORM3_HPP_
#define RIGID_BODY_KINEMATICS_TRANSFORM3_HPP_

#include <Eigen/Core>

#include "rigid_body_kinematics/lie_algebra.hpp"
#include "rigid_body_kinematics/numerical_policy.hpp"
#include "rigid_body_kinematics/rotation3.hpp"

namespace rigid_body_kinematics
{
enum class TransformLogBranch
{
  identity,
  pure_translation,
  small_angle,
  nominal,
  near_pi
};

struct TransformLogResult
{
  Vector6LinearFirst coordinates;
  TransformLogBranch branch;
};

class Transform3
{
public:
  // Create the identity transform
  static Transform3 identity();

  // Exponentiate finite linear-first coordinates [rho; phi].
  // rho is measured in metres and phi in radians.
  static Transform3 from_exponential_coordinates(
    const Vector6LinearFirst & coordinates,
    const NumericalPolicy & policy = NumericalPolicy{});

  // Recover principal finite linear-first coordinates [rho; phi].
  [[nodiscard]] TransformLogResult to_principal_exponential_coordinates(
    const NumericalPolicy & policy = NumericalPolicy{}) const;

  // Validate a raw homogeneous matrix and create a rigid transform
  static Transform3 from_matrix(
    const Eigen::Matrix4d & matrix,
    const NumericalPolicy & policy = NumericalPolicy{});

  // Transform point coordinates from frame {b} into frame {a}
  [[nodiscard]] Eigen::Vector3d transform_point(
    const Eigen::Vector3d & point_b) const;

  // Reconstruct the homogeneous matrix with exact bottom row.
  [[nodiscard]] Eigen::Matrix4d matrix() const;

  // Compose T_ab with T_bc to produce T_ac
  [[nodiscard]] Transform3 compose(const Transform3 & transform_bc) const;

  // Return the inverse transform T_ba
  [[nodiscard]] Transform3 inverse() const;

  // Integrate a constant space twist over a non-negative duration
  // The twist is ordered [linear velocity; angular velocity].
  [[nodiscard]] Transform3 integrate_constant_space_twist(
    const Vector6LinearFirst & space_twist, double delta_time,
    const NumericalPolicy & policy = NumericalPolicy{}) const;

  // Integrate a constant body twist over a non-negative duration.
  // The twist is ordered [linear velocity; angular velocity].
  [[nodiscard]] Transform3 integrate_constant_body_twist(
    const Vector6LinearFirst & body_twist, double delta_time,
    const NumericalPolicy & policy = NumericalPolicy{}) const;

  // Return the linear-first adjoint Ad_T.
  // It maps twist coordinates referenced to {b} into coordinates
  // referenced to {a} when this transform is T_ab.
  [[nodiscard]] Eigen::Matrix<double, 6, 6> adjoint() const;

private:
  Transform3(const Rotation3 & rotation, const Eigen::Vector3d & translation);

  Rotation3 rotation_;
  Eigen::Vector3d translation_;
};
}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_TRANSFORM3_HPP_
