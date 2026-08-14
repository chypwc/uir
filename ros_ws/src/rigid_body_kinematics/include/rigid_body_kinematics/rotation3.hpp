#ifndef RIGID_BODY_KINEMATICS__ROTATION3_HPP_
#define RIGID_BODY_KINEMATICS__ROTATION3_HPP_

#include <Eigen/Core>

#include "rigid_body_kinematics/numerical_policy.hpp"

namespace rigid_body_kinematics
{
enum class RotationLogBranch
{
  identity,
  small_angle,
  nominal,
  near_pi
};

struct RotationLogResult
{
  Eigen::Vector3d rotation_vector;
  RotationLogBranch branch;
};

class Rotation3
{
public:
  // Create identity rotation
  static Rotation3 identity();

  // Create a rotation after validating the supplied matrix
  static Rotation3 from_matrix(
    const Eigen::Matrix3d & matrix,
    const NumericalPolicy & policy = NumericalPolicy{});

  // Create a rotation from a finite rotation vector in radians.
  static Rotation3 from_rotation_vector(
    const Eigen::Vector3d & rotation_vector,
    const NumericalPolicy & policy = NumericalPolicy{});

  // Recover the deterministic principal rotation vector in radians
  [[nodiscard]] RotationLogResult to_principal_rotation_vector(
    const NumericalPolicy & policy = NumericalPolicy{}) const;

  // Apply this rotation to a vector
  [[nodiscard]] Eigen::Vector3d rotate_vector(
    const Eigen::Vector3d & vector_b) const;

  // Return the validated rotation matrix
  [[nodiscard]] Eigen::Matrix3d matrix() const;

  // Compose R_ab with R_bc to produce R_ac
  [[nodiscard]] Rotation3 compose(const Rotation3 & rotation_bc) const;

  // Return the inverse rotation R_ba
  [[nodiscard]] Rotation3 inverse() const;

private:
  // Construct from underlying matrix
  explicit Rotation3(const Eigen::Matrix3d & matrix);

  // Internal representation
  Eigen::Matrix3d matrix_;
};

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS__ROTATION3_HPP_
