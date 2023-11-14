cmake_minimum_required(VERSION 2.8.3)
project(backend)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)

catkin_install_python(PROGRAMS
    src/backend_node.py
    src/planner_emu.py
    src/google_map_to_occ_grid.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
