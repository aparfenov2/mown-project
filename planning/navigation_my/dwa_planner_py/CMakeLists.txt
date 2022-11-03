cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner_py)

find_package(catkin REQUIRED COMPONENTS
    pybind11_catkin)

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS pybind11_catkin
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${catkin_INCLUDE_DIRS})


pybind_add_module(dwa_planner_py MODULE src/dwa_planner_py.cpp)

catkin_install_python(PROGRAMS scripts/dwa_planner_test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS dwa_planner_py
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
