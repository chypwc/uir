#ifndef RIGID_BODY_KINEMATICS__TRANSFORM3_HPP_
#define RIGID_BODY_KINEMATICS__TRANSFORM3_HPP_

#include <Eigen/Core>

#include "rigid_body_kinematics/numerical_policy.hpp"
#include "rigid_body_kinematics/rotation3.hpp"

namespace rigid_body_kinematics
{
class Transform3
{
public:
  // Create the identity transform
  static Transform3 identity();

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

private:
  Transform3(const Rotation3 & rotation, const Eigen::Vector3d & translation);

  Rotation3 rotation_;
  Eigen::Vector3d translation_;
};
}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS__TRANSFORM3_HPP_