cmake_minimum_required(VERSION 3.12)

project(rigid_body_kinematics_visualization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rigid_body_kinematics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(spatial_2r_forward_kinematics_demo
  src/spatial_2r_forward_kinematics_demo.cpp
)

target_link_libraries(spatial_2r_forward_kinematics_demo
  PRIVATE rigid_body_kinematics::rigid_body_kinematics
)

install(
  TARGETS spatial_2r_forward_kinematics_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
