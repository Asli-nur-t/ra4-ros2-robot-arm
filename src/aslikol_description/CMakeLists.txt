cmake_minimum_required(VERSION 3.16)
project(aslikol_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
