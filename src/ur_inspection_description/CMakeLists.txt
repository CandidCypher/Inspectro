cmake_minimum_required(VERSION 3.22)
project(ur_inspection_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
