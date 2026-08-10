#include "rigid_body_kinematics/rotation3.hpp"

namespace rigid_body_kinematics
{
    Rotation3::Rotation3(const Eigen::Matrix3d &matrix)
        : matrix_(matrix) {}

    Rotation3 Rotation3::identity()
    {
        return Rotation3(Eigen::Matrix3d::Identity());
    }

    Eigen::Vector3d Rotation3::rotate_vector(
        const Eigen::Vector3d &vector_b) const
    {
        return matrix_ * vector_b;
    }

} // namespace rigid_body_kinematics