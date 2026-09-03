cmake_minimum_required(VERSION 3.12)

# Define the project name and specify that this project uses C++.
project(rigid_body_kinematics LANGUAGES CXX)

# Compile package targets using ISO C++ rather than compiler-specific extensions.
set(CMAKE_CXX_EXTENSIONS OFF)

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
  src/transform3.cpp
  src/lie_algebra.cpp
  src/planar_geometry.cpp
  src/joint_limits.cpp
  src/joint_definition.cpp
  src/serial_chain_model.cpp
  src/forward_kinematics.cpp
)


# Create a namespaced alias for the same library target.
# This allows it to be referred to as:
# rigid_body_kinematics::rigid_body_kinematics
add_library(${PROJECT_NAME}::${PROJECT_NAME}
  ALIAS ${PROJECT_NAME}
)


# Require C++20 for this library and for targets that depend on it.
target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_20
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

  ament_add_gtest(test_transform3
    test/test_transform3.cpp
  )

  target_link_libraries(test_transform3
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_lie_algebra
    test/test_lie_algebra.cpp
  )

  target_link_libraries(test_lie_algebra
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_so3_exponential
    test/test_so3_exponential.cpp
  )

  target_link_libraries(test_so3_exponential
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_so3_logarithm
    test/test_so3_logarithm.cpp
  )

  target_link_libraries(test_so3_logarithm
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_se3_exponential
    test/test_se3_exponential.cpp
  )

  target_link_libraries(test_se3_exponential
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_se3_logarithm
    test/test_se3_logarithm.cpp
  )

  target_link_libraries(test_se3_logarithm
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_constant_twist_integration
    test/test_constant_twist_integration.cpp
  )

  target_link_libraries(test_constant_twist_integration
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_se3_adjoint
    test/test_se3_adjoint.cpp
  )

  target_link_libraries(test_se3_adjoint
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_planar_geometry
    test/test_planar_geometry.cpp
  )

  target_link_libraries(test_planar_geometry
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_joint_limits
    test/test_joint_limits.cpp
  )

  target_link_libraries(test_joint_limits
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_joint_definition
    test/test_joint_definition.cpp
  )

  target_link_libraries(test_joint_definition
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_serial_chain_model
    test/test_serial_chain_model.cpp
  )

  target_link_libraries(test_serial_chain_model
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_forward_kinematics
    test/test_forward_kinematics.cpp
  )

  target_link_libraries(test_forward_kinematics
    ${PROJECT_NAME}
  )

  # Enable automatic ROS 2 linting support.
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_test REQUIRED)

  find_program(
    PROJECT_CPPLINT_EXECUTABLE
    NAMES cpplint
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../.venv/bin"
    NO_DEFAULT_PATH
  )

  if(NOT PROJECT_CPPLINT_EXECUTABLE)
    message(FATAL_ERROR
      "cpplint 2.0.2 was not found in the project virtual environment. "
      "Install requirements-dev.txt before configuring tests."
    )
  endif()

  # Do not let ament_lint_auto register these checks automatically:
  # - copyright is intentionally skipped;
  # - cpplint and clang-format are registered manually below;
  # - uncrustify is replaced by clang-format.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  # Find and register lint test dependencies declared by the package.
  ament_lint_auto_find_test_dependencies()

  ament_clang_format(
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format"
    include
    src
    test
  )

  # cpplint
  set(PROJECT_CPPLINT_FILTERS
    -legal/copyright
    -runtime/references
    -whitespace/braces
    -whitespace/indent
    -whitespace/parens
    -whitespace/semicolon
  )

  list(
    JOIN PROJECT_CPPLINT_FILTERS
    ","
    PROJECT_CPPLINT_FILTER_ARGUMENT
  )

  ament_add_test(
    cpplint
    COMMAND
      "${PROJECT_CPPLINT_EXECUTABLE}"
      --recursive
      "--root=${CMAKE_CURRENT_SOURCE_DIR}/include"
      --linelength=100
      "--filter=${PROJECT_CPPLINT_FILTER_ARGUMENT}"
      include
      src
      test
    OUTPUT_FILE
      "${CMAKE_CURRENT_BINARY_DIR}/ament_cpplint/cpplint.txt"
    WORKING_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 120
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  set_tests_properties(
    cpplint
    PROPERTIES LABELS "cpplint;linter"
  )
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


# Install the vendored ROS package manifest schema so the installed
# package.xml can also be validated without network access.
install(
  DIRECTORY schema/
  DESTINATION share/${PROJECT_NAME}/schema/
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
