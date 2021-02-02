cmake_minimum_required(VERSION 3.0.2)
project(move_base_mod)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        cmake_modules
        costmap_2d
#        dynamic_reconfigure
        actionlib_msgs
        geometry_msgs
        message_generation
#        move_base_msgs
        nav_core
        nav_msgs
#        navfn
        pluginlib
        roscpp
        rospy
#        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/MoveBase.cfg
# )

# actions
add_action_files(
  DIRECTORY
    action
  FILES
    TrajectoryController.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        # dynamic_reconfigure
        geometry_msgs
        actionlib_msgs
        message_runtime
        # move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base_mod
add_library(move_base_mod
  src/move_base_mod.cpp
)
target_link_libraries(move_base_mod
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(move_base_mod ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_base_node
  src/move_base_node.cpp
)
add_dependencies(move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node move_base_mod)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base_mod)

install(
    TARGETS
        move_base_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        move_base_mod
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
