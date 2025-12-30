# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/agarre_gazebo/CMakeLists.txt
# Summary: CMake build config for agarre_gazebo package.
cmake_minimum_required(VERSION 3.8)
project(agarre_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip lint checks for this delivery workspace.
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
