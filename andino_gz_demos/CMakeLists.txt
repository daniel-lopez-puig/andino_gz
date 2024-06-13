cmake_minimum_required(VERSION 3.7)
project(andino_gz_demos)

find_package(gazebo QUIET)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    maps
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
