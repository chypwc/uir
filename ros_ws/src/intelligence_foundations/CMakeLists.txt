cmake_minimum_required(VERSION 3.12)

project(intelligence_foundations LANGUAGES CXX)

# Compile project-owned targets as ISO C++ without compiler extensions.
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# The first capability is header-only. This target will become a compiled
# library when the finite-MDP implementation gains source files.
add_library(${PROJECT_NAME}
  src/joint_outcome_distribution.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME}
  ALIAS ${PROJECT_NAME}
)

# Require C++20 for this target and every downstream consumer.
target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_20
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_test REQUIRED)

  ament_add_gtest(test_state_action_index
    test/test_state_action_index.cpp
  )

  target_link_libraries(test_state_action_index
    ${PROJECT_NAME}
  )

  target_compile_options(test_state_action_index
    PRIVATE
      "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>"
  )

  ament_add_gtest(test_joint_outcome_distribution
    test/test_joint_outcome_distribution.cpp
  )

  target_link_libraries(test_joint_outcome_distribution
    ${PROJECT_NAME}
  )

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

  # Use the repository formatter and local cpplint instead of ament's
  # automatic formatting, copyright, and cpplint checks.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  ament_clang_format(
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format"
    include
    src
    test
  )

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

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY schema/
  DESTINATION share/${PROJECT_NAME}/schema/
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_package()
