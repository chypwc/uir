#ifndef RIGID_BODY_KINEMATICS_NUMERICAL_POLICY_HPP_
#define RIGID_BODY_KINEMATICS_NUMERICAL_POLICY_HPP_

namespace rigid_body_kinematics
{
struct NumericalPolicy
{
  double orthogonality_tolerance{1.0e-12};
  double determinant_tolerance{1.0e-12};
  double homogeneous_row_tolerance{1.0e-12};
  double tangent_tolerance{1.0e-12};

  // Inclusive upper bound for the small-angle series
  double series_angle_threshold{1.0e-4};

  // Maximum supported rotation-vector norm in radians.
  double maximum_exponential_angle{1.0e6};

  // Inclusive distance from pi for near-pi axis recovery.
  double near_pi_tolerance{1.0e-6};

  // Dimensionless tolerance for preserving the positive z-axis:
  // || R e_z - e_z || <= planar_orientation_tolerance
  double planar_orientation_tolerance{1.0e-12};

  // Vertical-position tolerance in metres:
  // |p_z| <= planar_position_tolerance
  double planar_position_tolerance{1.0e-12};
};

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_NUMERICAL_POLICY_HPP_
