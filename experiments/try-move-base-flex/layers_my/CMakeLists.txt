cmake_minimum_required(VERSION 2.8.3)
project(layers_my)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             std_msgs
             tf
             tf2
             tf_conversions
             robot_localization
             parameter_assertions
             costmap_2d
             dynamic_reconfigure
             pluginlib
    )

catkin_package(
    CATKIN_DEPENDS std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(rolling_layer
        src/rolling_layer.cpp
        src/rolling_layer_config.cpp
        )
add_dependencies(rolling_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_layer ${catkin_LIBRARIES})

add_library(unrolling_layer
    src/unrolling_layer.cpp
    src/unrolling_layer_config.cpp
    )
add_dependencies(unrolling_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(unrolling_layer ${catkin_LIBRARIES})

install(
    TARGETS rolling_layer unrolling_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
