#pragma once

#include "rclcpp/time.hpp"
#include "rigid_body_kinematics/serial_chain_model.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rigid_body_kinematics_visualization
{

visualization_msgs::msg::MarkerArray make_spatial_2r_markers(
  const rigid_body_kinematics::SerialChainModel & elbow_model,
  const rigid_body_kinematics::SerialChainModel & end_model, double u,
  const rclcpp::Time & stamp);

}  // namespace rigid_body_kinematics_visualization
