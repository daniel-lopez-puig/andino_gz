cmake_minimum_required(VERSION 3.7)
project(andino_gz)

find_package(gazebo QUIET)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    rviz
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  enable_testing()
  ament_flake8("--config" ${CMAKE_CURRENT_SOURCE_DIR}/.flake8)
endif()

ament_package()
