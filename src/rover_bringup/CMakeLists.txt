cmake_minimum_required(VERSION 3.8)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config description maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
