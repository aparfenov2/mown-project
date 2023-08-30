cmake_minimum_required(VERSION 2.8.3)
project(pidnet_py)

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()
catkin_install_python(PROGRAMS
    pidnet_node.py
    pidnet_tool.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
