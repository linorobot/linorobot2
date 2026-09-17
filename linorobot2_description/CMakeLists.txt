cmake_minimum_required(VERSION 3.8...3.25)
project(linorobot2_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
