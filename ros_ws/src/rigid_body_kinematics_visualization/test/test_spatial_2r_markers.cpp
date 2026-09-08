#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/transform3.hpp"
#include "rigid_body_kinematics_visualization/spatial_2r_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{
constexpr double kPositionToleranceMetres = 1.0e-12;

void expect_point_near(
  const geometry_msgs::msg::Point & actual,
  const Eigen::Vector3d & expected_metres)
{
  const Eigen::Vector3d actual_metres(actual.x, actual.y, actual.z);
  EXPECT_LE((actual_metres - expected_metres).norm(), kPositionToleranceMetres);
}
}  // namespace

// Verify world-frame joint centres, link endpoints, and axis-arrow
// endpoints at home and at the independently known bent configuration.
// Arrow tip_i = p_e + 0.3 R_se e_i, with lengths in metres.
TEST(Spatial2RMarkersTest, HomeAndBentGeometryMatchAnalyticResults)
{
  namespace rbk = rigid_body_kinematics;
  using Marker = visualization_msgs::msg::Marker;
  using rigid_body_kinematics_visualization::make_spatial_2r_markers;

  const auto limits =
    rbk::RevoluteLimits::from_bounds(-std::numbers::pi, std::numbers::pi);

  const auto joint_1 = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), limits);
  const auto joint_2 = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d::UnitY(), Eigen::Vector3d(2.0, 0.0, 0.0), limits);

  Eigen::Matrix4d elbow_home = Eigen::Matrix4d::Identity();
  elbow_home(0, 3) = 2.0;

  Eigen::Matrix4d end_home = Eigen::Matrix4d::Identity();
  end_home(0, 3) = 3.0;

  const auto elbow_model = rbk::SerialChainModel::from_home_and_joints(
    rbk::Transform3::from_matrix(elbow_home), {joint_1});
  const auto end_model = rbk::SerialChainModel::from_home_and_joints(
    rbk::Transform3::from_matrix(end_home), {joint_1, joint_2});

  // A fixed timestamp of zero.
  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);

  for (const double u : std::array<double, 2>{0.0, 1.0}) {
    SCOPED_TRACE(u);

    std::array<Eigen::Vector3d, 3> expected_positions{
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
    };
    Eigen::Matrix3d expected_rotation = Eigen::Matrix3d::Identity();

    if (u == 1.0) {
      expected_positions[1] = Eigen::Vector3d(0.0, 2.0, 0.0);
      expected_positions[2] = Eigen::Vector3d(0.0, 2.0, 1.0);
      // clang-format off
      expected_rotation <<
        0.0, -1.0,  0.0,
        0.0,  0.0, -1.0,
        1.0,  0.0,  0.0;
      // clang-format on
    }

    const auto message =
      make_spatial_2r_markers(elbow_model, end_model, u, stamp);

    // Find markers by their identity, not their array position.
    const auto find_marker = [&message](int id) {
      return std::find_if(
        message.markers.begin(), message.markers.end(),
        [id](const Marker & marker) {
          return marker.ns == "spatial_2r_fk" && marker.id == id;
        });
    };

    // World-coordinate points must not receive a second pose transform.
    for (int id = 0; id <= 4; ++id) {
      const auto marker = find_marker(id);
      ASSERT_NE(marker, message.markers.end());

      EXPECT_EQ(marker->header.frame_id, "world");
      expect_point_near(marker->pose.position, Eigen::Vector3d::Zero());
      EXPECT_DOUBLE_EQ(marker->pose.orientation.x, 0.0);
      EXPECT_DOUBLE_EQ(marker->pose.orientation.y, 0.0);
      EXPECT_DOUBLE_EQ(marker->pose.orientation.z, 0.0);
      EXPECT_DOUBLE_EQ(marker->pose.orientation.w, 1.0);

      if (id <= 1) {
        EXPECT_EQ(
          marker->type, id == 0 ? Marker::SPHERE_LIST : Marker::LINE_STRIP);
        ASSERT_EQ(marker->points.size(), 3U);

        for (std::size_t index = 0; index < 3; ++index) {
          expect_point_near(marker->points[index], expected_positions[index]);
        }
      } else {
        EXPECT_EQ(marker->type, Marker::ARROW);
        ASSERT_EQ(marker->points.size(), 2U);

        const Eigen::Vector3d expected_tip =
          expected_positions[2] + 0.3 * expected_rotation.col(id - 2);

        expect_point_near(marker->points[0], expected_positions[2]);
        expect_point_near(marker->points[1], expected_tip);
      }
    }
  }
}

// Invalid u must throw instead of returning a marker array.
// Check both interval violations and non-finite values.
TEST(Spatial2RMarkersTest, RejectsInvalidMotionParameter)
{
  namespace rbk = rigid_body_kinematics;
  using rigid_body_kinematics_visualization::make_spatial_2r_markers;

  const auto limits =
    rbk::RevoluteLimits::from_bounds(-std::numbers::pi, std::numbers::pi);

  const auto joint_1 = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), limits);
  const auto joint_2 = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d::UnitY(), Eigen::Vector3d(2.0, 0.0, 0.0), limits);

  Eigen::Matrix4d elbow_home = Eigen::Matrix4d::Identity();
  elbow_home(0, 3) = 2.0;

  Eigen::Matrix4d end_home = Eigen::Matrix4d::Identity();
  end_home(0, 3) = 3.0;

  const auto elbow_model = rbk::SerialChainModel::from_home_and_joints(
    rbk::Transform3::from_matrix(elbow_home), {joint_1});
  const auto end_model = rbk::SerialChainModel::from_home_and_joints(
    rbk::Transform3::from_matrix(end_home), {joint_1, joint_2});

  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);

  const std::array<double, 5> invalid_parameters{
    -0.01,
    1.01,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
  };

  for (const double u : invalid_parameters) {
    SCOPED_TRACE(u);

    EXPECT_THROW(
      make_spatial_2r_markers(elbow_model, end_model, u, stamp),
      std::invalid_argument);
  }
}
