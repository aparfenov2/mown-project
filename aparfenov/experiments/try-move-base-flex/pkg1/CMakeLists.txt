cmake_minimum_required(VERSION 2.8.3)
project(pkg1)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/move_base_legacy_relay.py
    scripts/coverage_progress
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
