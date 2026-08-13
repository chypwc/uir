#ifndef RIGID_BODY_KINEMATICS__NUMERICAL_POLICY_HPP_
#define RIGID_BODY_KINEMATICS__NUMERICAL_POLICY_HPP_

namespace rigid_body_kinematics
{
struct NumericalPolicy
{
  double orthogonality_tolerance{1.0e-12};
  double determinant_tolerance{1.0e-12};
  double homogeneous_row_tolerance{1.0e-12};
};

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS__NUMERICAL_POLICY_HPP_
