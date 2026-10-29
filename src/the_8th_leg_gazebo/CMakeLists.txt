cmake_minimum_required(VERSION 3.10)
project(the_8th_leg_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
