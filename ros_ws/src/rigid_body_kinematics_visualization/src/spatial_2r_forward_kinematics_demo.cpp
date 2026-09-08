#include <Eigen/Core>
#include <chrono>  // time durations: std::chrono::milliseconds
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>  // std::setprecision
#include <iostream>
#include <memory>  // smart pointers: std::shared_ptr, std::make_shared
#include <numbers>
#include <sstream>  // std::ostringstream
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"  // geometry_msgs::msg::Point
#include "rclcpp/rclcpp.hpp"            // ROS 2 C++ interface
#include "rigid_body_kinematics/forward_kinematics.hpp"
#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/serial_chain_model.hpp"
#include "rigid_body_kinematics/transform3.hpp"
#include "rigid_body_kinematics_visualization/spatial_2r_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"  // Describe a visual object for RViz
#include "visualization_msgs/msg/marker_array.hpp"  // a collection of markers

int main(int argc, char * argv[])
{
  namespace rbk = rigid_body_kinematics;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using rigid_body_kinematics_visualization::make_spatial_2r_markers;

  try {
    // Initialize ROS and create the node
    rclcpp::init(argc, argv);

    // Creates a node named spatial_2r_forward_kinematics_demo.
    const auto node =
      std::make_shared<rclcpp::Node>("spatial_2r_forward_kinematics_demo");

    // Print a ROS log message using that node's logger.
    RCLCPP_INFO(node->get_logger(), "Spatial 2R node started.");

    /*
    Compute end effector's pose.
    */
    const double first_link_length_metres = 2.0;
    const double second_link_length_metres = 1.0;

    const rbk::RevoluteLimits limits =
      rbk::RevoluteLimits::from_bounds(-std::numbers::pi, std::numbers::pi);

    // Both axis descriptions refer to the home configuration.
    const rbk::RevoluteJoint joint_1 = rbk::RevoluteJoint::from_axis(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), limits);

    const rbk::RevoluteJoint joint_2 = rbk::RevoluteJoint::from_axis(
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d(first_link_length_metres, 0.0, 0.0), limits);

    // Each endpoint has its own home pose.
    Eigen::Matrix4d elbow_home_matrix = Eigen::Matrix4d::Identity();
    elbow_home_matrix(0, 3) = first_link_length_metres;

    Eigen::Matrix4d end_home_matrix = Eigen::Matrix4d::Identity();
    end_home_matrix(0, 3) =
      first_link_length_metres + second_link_length_metres;

    const rbk::SerialChainModel elbow_model =
      rbk::SerialChainModel::from_home_and_joints(
        rbk::Transform3::from_matrix(elbow_home_matrix), {joint_1});

    const rbk::SerialChainModel end_model =
      rbk::SerialChainModel::from_home_and_joints(
        rbk::Transform3::from_matrix(end_home_matrix), {joint_1, joint_2});

    const MarkerArray markers =
      make_spatial_2r_markers(elbow_model, end_model, 1.0, node->now());

    std::cout << "Prepared " << markers.markers.size() << " marker messages.\n";

    /*
    Create a publisher.
    MarkerArray: the message type this publisher sends.
    "~/markers": topic name relative to the node's name. With our default node name,
            it becomes /spatial_2r_forward_kinematics_demo/markers.
    QoS(1): heeps a history of one message.
    reliable(): requests reliable delivery.
    durability_volatile(): does not retail old message for subscribers. Our repeated
                          publication supplies fresh ones.
    */
    const auto publisher = node->create_publisher<MarkerArray>(
      "~/markers", rclcpp::QoS(1).reliable().durability_volatile());

    /*
    Add a timer.
    Each callback gives both markers the same current timestamp,
    then publishes the complete array.
    Keeping timer in a local variable keeps the timer alive while spin(node) runs.
    The timer requests a callback every 50ms, or 20 times per second.
    node, publisher: shared pointers, keeping those objects available.
    */

    const bool animate = node->declare_parameter<bool>("animate", false);

    const auto start_time = std::chrono::steady_clock::now();

    const auto timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      [node, publisher, elbow_model, end_model, animate, start_time]() {
        double u = 1.0;

        if (animate) {
          const double elapsed_seconds =
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - start_time)
              .count();

          const double cycle_seconds = std::fmod(elapsed_seconds, 8.0);

          u = cycle_seconds < 4.0 ? cycle_seconds / 4.0
                                  : (8.0 - cycle_seconds) / 4.0;
        }

        const auto message =
          make_spatial_2r_markers(elbow_model, end_model, u, node->now());

        publisher->publish(message);
      });

    /*
    Keep the node running
    */
    rclcpp::spin(node);  // Keep processing ROS events until shutdown.
    rclcpp::shutdown();
    return EXIT_SUCCESS;

  } catch (const std::exception & error) {
    std::cerr << "Spatial 2R evaluation failed: " << error.what() << '\n';

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    return EXIT_FAILURE;
  }
}
