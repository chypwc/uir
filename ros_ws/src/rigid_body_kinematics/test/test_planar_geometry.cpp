#include <gtest/gtest.h>

#include "rigid_body_kinematics/planar_geometry.hpp"

TEST(PlanarGeometryTest, AcceptanceCaseEmbedsQuarterTurnPose)
{
  constexpr double kPi = 3.141592653589793238462643383279502884;

  const rigid_body_kinematics::PlanarPose planar_pose{2.0, -1.0, kPi / 2.0};

  Eigen::Matrix4d actual =
    rigid_body_kinematics::embed_planar_pose(planar_pose).matrix();

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
    0.0, -1.0, 0.0,  2.0,
    1.0,  0.0, 0.0, -1.0,
    0.0,  0.0, 1.0,  0.0,
    0.0,  0.0, 0.0,  1.0;
  // clang-format on

  constexpr double tolerance = 1.0e-12;
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      EXPECT_NEAR(actual(row, column), expected(row, column), tolerance);
    }
  }
}
