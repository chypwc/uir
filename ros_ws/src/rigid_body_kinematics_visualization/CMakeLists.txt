cmake_minimum_required(VERSION 3.12)

project(rigid_body_kinematics_visualization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rigid_body_kinematics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(spatial_2r_forward_kinematics_demo
  src/spatial_2r_forward_kinematics_demo.cpp
  src/spatial_2r_markers.cpp
)

target_include_directories(spatial_2r_forward_kinematics_demo
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(spatial_2r_forward_kinematics_demo
  PUBLIC
    rclcpp
    geometry_msgs
    visualization_msgs
)

target_link_libraries(spatial_2r_forward_kinematics_demo
  PRIVATE rigid_body_kinematics::rigid_body_kinematics
)

install(
  TARGETS spatial_2r_forward_kinematics_demo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_spatial_2r_markers
    test/test_spatial_2r_markers.cpp
    src/spatial_2r_markers.cpp
  )

  target_include_directories(test_spatial_2r_markers
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_target_dependencies(test_spatial_2r_markers
      rclcpp
      geometry_msgs
      visualization_msgs
  )

  target_link_libraries(test_spatial_2r_markers
    rigid_body_kinematics::rigid_body_kinematics
  )
endif()

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
