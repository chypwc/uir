#include <gtest/gtest.h>
#include <Eigen/Core>
#include "rigid_body_kinematics/rotation3.hpp"

TEST(Rotation3Test, IdentityLeavesFreeVectorUnchanged)
{
    const rigid_body_kinematics::Rotation3 rotation =
        rigid_body_kinematics::Rotation3::identity();

    const Eigen::Vector3d vector_b(1.0, -2.0, 3.5);

    const Eigen::Vector3d vector_a =
        rotation.rotate_vector(vector_b);

    EXPECT_DOUBLE_EQ(vector_a.x(), 1.0);
    EXPECT_DOUBLE_EQ(vector_a.y(), -2.0);
    EXPECT_DOUBLE_EQ(vector_a.z(), 3.5);
}