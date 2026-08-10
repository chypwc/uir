#ifndef RIGID_BODY_KINEMATICS_ROTATION3_HPP_
#define RIGID_BODY_KINEMATICS_ROTATION3_HPP_

#include <Eigen/Core>

namespace rigid_body_kinematics
{

    class Rotation3
    {
    public:
        // Create identity rotation
        static Rotation3 identity();

        // Apply this rotation to a vector
        [[nodiscard]] Eigen::Vector3d rotate_vector(
            const Eigen::Vector3d &vector_b) const;

    private:
        // Construct from underlying matrix
        explicit Rotation3(const Eigen::Matrix3d &matrix);

        // Internal representation
        Eigen::Matrix3d matrix_;
    };

} // namespace rigid_body_kinematics

#endif