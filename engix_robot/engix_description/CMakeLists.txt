cmake_minimum_required(VERSION 2.8.3)
project(engix_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch mesh urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)