#include "rigid_body_kinematics_visualization/spatial_2r_markers.hpp"

#include <Eigen/Core>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/duration.hpp"
#include "rigid_body_kinematics/forward_kinematics.hpp"
#include "rigid_body_kinematics/transform3.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

geometry_msgs::msg::Point to_point_message(
  const Eigen::Vector3d & position_metres)
{
  geometry_msgs::msg::Point point;
  point.x = position_metres.x();
  point.y = position_metres.y();
  point.z = position_metres.z();
  return point;
}

}  // namespace

namespace rigid_body_kinematics_visualization
{

visualization_msgs::msg::MarkerArray make_spatial_2r_markers(
  const rigid_body_kinematics::SerialChainModel & elbow_model,
  const rigid_body_kinematics::SerialChainModel & end_model, double u,
  const rclcpp::Time & stamp)
{
  namespace rbk = rigid_body_kinematics;

  if (!std::isfinite(u) || u < 0.0 || u > 1.0) {
    throw std::invalid_argument(
      "The motion parameter u must be finite and within [0, 1].");
  }

  // Move the existing coordinate, pose, and marker code.

  Eigen::VectorXd joint_coordinates(2);
  joint_coordinates << u * std::numbers::pi / 2.0, -u * std::numbers::pi / 2.0;

  Eigen::VectorXd elbow_coordinates(1);
  elbow_coordinates(0) = joint_coordinates(0);

  const rbk::Transform3 elbow_pose =
    rbk::space_form_forward_kinematics(elbow_model, elbow_coordinates);

  const rbk::Transform3 end_pose =
    rbk::space_form_forward_kinematics(end_model, joint_coordinates);

  const Eigen::Vector3d elbow_position_metres =
    elbow_pose.transform_point(Eigen::Vector3d::Zero());

  /*
    Create marker message describing spheres on the base, elbow, and end effector
    and lines connecting them.
    */
  const Eigen::Vector3d end_position_metres =
    end_pose.transform_point(Eigen::Vector3d::Zero());

  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  // Describe spheres at the base, elbow, and end effector.
  Marker joints;
  joints.header.frame_id = "world";
  joints.header.stamp = stamp;

  // // namespace: a string label grouping related markers
  // ("spatial_2r_fk", 0) → joint spheres
  // ("spatial_2r_fk", 1) → connecting links
  joints.ns = "spatial_2r_fk";
  joints.id = 0;
  joints.type = Marker::SPHERE_LIST;
  joints.action = Marker::ADD;

  // ROS stores orientation as a quaternion, with four components: x, y, z, w.
  // For a rotation by angle θ about a unit axis a, a unit quaternion has
  // [x, y, z] = a sin(θ/2); w = cos(θ/2).
  // For zero rotation θ=0, (x, y, z, w) = (0, 0, 0, 1).
  joints.pose.orientation.w = 1.0;

  // Make the spheres' diameter 0.12m.
  joints.scale.x = 0.12;
  joints.scale.y = 0.12;
  joints.scale.z = 0.12;

  // Select an orange color.
  joints.color.r = 1.0F;
  joints.color.g = 0.6F;
  joints.color.b = 0.1F;
  joints.color.a = 1.0F;  // Fully opaque.

  // The lifetime is 0.5s. Later, repeated publication will refresh the markers.
  joints.lifetime = rclcpp::Duration::from_seconds(0.5);

  joints.points = {
    to_point_message(Eigen::Vector3d::Zero()),
    to_point_message(elbow_position_metres),
    to_point_message(end_position_metres),
  };

  // Describe links connecting those same ordered positions.
  // Both messages need the same frame, timestamp, identity pose, and positions.
  // Copying retains those settings; then we change the ID, shape, width, and colour.
  Marker links = joints;
  links.id = 1;

  // LINE_STRIP connects consecutive points: base -> elbow -> end effector
  links.type = Marker::LINE_STRIP;
  links.scale.x = 0.05;  // control line width of 0.05m

  links.color.r = 0.2F;
  links.color.g = 0.7F;
  links.color.b = 1.0F;

  MarkerArray markers;
  markers.markers = {joints, links};

  // Add three arrows showing the end-effector's orientation
  const Eigen::Matrix3d end_rotation = end_pose.matrix().topLeftCorner<3, 3>();
  const double axis_length_metres = 0.3;

  for (int axis_index = 0; axis_index < 3; ++axis_index) {
    const Eigen::Vector3d tip_position_metres =
      end_position_metres + axis_length_metres * end_rotation.col(axis_index);

    Marker axis = joints;
    axis.id = 2 + axis_index;
    axis.type = Marker::ARROW;

    axis.scale.x = 0.025;  // Shaft diameter [m].
    axis.scale.y = 0.05;   // Head diameter [m].
    axis.scale.z = 0.08;   // Head length [m].

    // x: red; y: green;
    axis.color.r = axis_index == 0 ? 1.0F : 0.0F;
    axis.color.b = 0.0F;
    axis.color.g = axis_index == 1 ? 1.0F : 0.0F;
    // z: light blue

    if (axis_index == 2) {
      // axis.color.r = 50.0F / 255.0F;
      axis.color.g = 220.0F / 255.0F;
      axis.color.b = 255.0F / 255.0F;
      axis.color.a = 1.0F;
    }

    axis.color.a = 1.0F;

    axis.points = {
      to_point_message(end_position_metres),
      to_point_message(tip_position_metres),
    };

    markers.markers.push_back(axis);

    // Add axis label. An extra 0.08m beyond the arrow tip.
    const Eigen::Vector3d axis_label_position_metres =
      tip_position_metres + 0.08 * end_rotation.col(axis_index);

    Marker axis_label = axis;
    axis_label.id = 7 + axis_index;
    axis_label.type = Marker::TEXT_VIEW_FACING;
    axis_label.points.clear();

    axis_label.pose.position = to_point_message(axis_label_position_metres);
    axis_label.scale.z = 0.10;
    axis_label.text = "xyz"[axis_index];

    markers.markers.push_back(axis_label);
  }

  // Add text marker
  // stores the text in memory
  std::ostringstream pose_text;
  pose_text << std::fixed << std::setprecision(3) << "q [rad]: ("
            << joint_coordinates(0) << ", " << joint_coordinates(1) << ")\n"
            << "p_e in world [m]: (" << end_position_metres.x() << ", "
            << end_position_metres.y() << ", " << end_position_metres.z()
            << ")";

  Marker label = joints;
  label.id = 5;
  // makes the text face the camera
  label.type = Marker::TEXT_VIEW_FACING;
  // removes the three sphere positions copied from joints
  label.points.clear();

  // position label
  label.pose.position = to_point_message(end_position_metres);
  label.pose.position.z += 0.6;
  label.scale.z = 0.11;  // control text size

  // White color.
  label.color.r = 1.0F;
  label.color.g = 1.0F;
  label.color.b = 1.0F;
  label.color.a = 1.0F;

  label.text = pose_text.str();
  markers.markers.push_back(label);

  return markers;
}

}  // namespace rigid_body_kinematics_visualization
