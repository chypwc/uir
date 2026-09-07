cmake_minimum_required(VERSION 3.12)

project(rigid_body_kinematics_visualization LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rigid_body_kinematics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_package()
