cmake_minimum_required(VERSION 3.8)
project(microspot_description)


find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
