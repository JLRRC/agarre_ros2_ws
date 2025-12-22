# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_description/CMakeLists.txt
# Summary: CMake build config for ur5_description package.
cmake_minimum_required(VERSION 3.8)
project(ur5_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
