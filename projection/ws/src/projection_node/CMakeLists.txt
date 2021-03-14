cmake_minimum_required(VERSION 2.8.3)
project(projection_node)

find_package(catkin REQUIRED COMPONENTS
  elevation_mapping
  grid_map_rviz_plugin
  rviz
)
catkin_package(
  CATKIN_DEPENDS
    elevation_mapping
    grid_map_rviz_plugin
    rviz
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
