cmake_minimum_required(VERSION 3.8...3.25)
project(linorobot2_base)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
