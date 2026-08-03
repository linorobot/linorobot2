cmake_minimum_required(VERSION 3.5)
project(linorobot2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/map_viewer.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
