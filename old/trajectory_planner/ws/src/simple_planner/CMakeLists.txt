cmake_minimum_required(VERSION 2.8.3)
project(simple_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/simple_planner.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
