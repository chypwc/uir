cmake_minimum_required(VERSION 3.8)

# Define the project name and specify that this project uses C++.
project(rigid_body_kinematics LANGUAGES CXX)

# Find the ROS 2 ament CMake build system.
find_package(ament_cmake REQUIRED)

# Find ROS 2's Eigen CMake support module: helps CMake/ROS locate Eigen
find_package(eigen3_cmake_module REQUIRED)

# Find the Eigen3 linear algebra library.
find_package(Eigen3 REQUIRED)


# Create a library named rigid_body_kinematics from the source file.
# src/rotation3.cpp
#         ↓ compile
# rotation3.o
#         ↓ link
# rigid_body_kinematics library, like librigid_body_kinematics.so
add_library(${PROJECT_NAME}
  src/rotation3.cpp
)

# Create a namespaced alias for the same library target.
# This allows it to be referred to as:
# rigid_body_kinematics::rigid_body_kinematics
add_library(${PROJECT_NAME}::${PROJECT_NAME}
  ALIAS ${PROJECT_NAME}
)


# Require C++17 for this library and for targets that depend on it.
target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_17
)


# Enable additional compiler warnings when using GCC or Clang.
# PRIVATE means these warning flags apply only when compiling this library
# and are not propagated to downstream targets.
# 
# IF:
#     language is C++
#     AND compiler is GNU or Clang
# THEN:
#     use -Wall
#     use -Wextra
#     use -Wpedantic
target_compile_options(${PROJECT_NAME}
  PRIVATE
    "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>"
)


# Specify where the compiler should look for this library's header files.
target_include_directories(${PROJECT_NAME}
  PUBLIC

    # While building this package, use the include/ directory
    # in the current source tree.
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

    # After installation, consumers should use the installed include/ directory.
    $<INSTALL_INTERFACE:include>
)


# Declare Eigen as a public dependency.
# This library uses Eigen, and its public headers also expose Eigen types,
# so downstream targets need access to Eigen as well.
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
)


# Configure test-related dependencies only when testing is enabled.
if(BUILD_TESTING)

  # Provide GoogleTest integration for ROS 2 / ament.
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rotation3
    test/test_rotation3.cpp 
  )

  target_link_libraries(test_rotation3
    ${PROJECT_NAME} 
  )

  # Enable automatic ROS 2 linting support.
  find_package(ament_lint_auto REQUIRED)

  # Find and register lint test dependencies declared by the package.
  ament_lint_auto_find_test_dependencies()

endif()


# Install the compiled library.
install(
  TARGETS ${PROJECT_NAME}

  # Add this target to an export set so other packages can import it.
  EXPORT export_${PROJECT_NAME}

  # Static libraries (.a) are installed into lib/.
  ARCHIVE DESTINATION lib

  # Shared libraries (.so) are installed into lib/.
  LIBRARY DESTINATION lib

  # Runtime binaries, such as Windows DLL-related executables,
  # are installed into bin/.
  RUNTIME DESTINATION bin
)


# Install the public header files.
install(
  DIRECTORY include/
  DESTINATION include/
)


# Export the installed CMake target so downstream ROS 2 packages
# can use this library.
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)


# Export Eigen as a dependency of this package so downstream
# packages know that Eigen must also be available.
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
)


# Finalize the ROS 2 ament package and generate/install package metadata.
ament_package()