cmake_minimum_required(VERSION 2.8.3)
project(engix_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  nav_msgs
  tf
  diagnostic_updater
  cv_bridge
  image_transport
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  odrive_msg.msg
  odrive_ctrl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(usb-1.0_LIBRARIES usb-1.0)
set(jsoncpp_LIBRARIES jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs nav_msgs
  DEPENDS ${rt_LIBRARIES} ${pthread_LIBRARIES} ${usb-1.0_LIBRARIES} ${jsoncpp_LIBRARIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(engix_robot src/odrive.cpp src/odrive_endpoint.cpp src/odrive_utils.cpp)
add_dependencies(engix_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(engix_robot ${usb-1.0_LIBRARIES} ${catkin_LIBRARIES} ${jsoncpp_LIBRARIES})

add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)
add_dependencies(robot_pose_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_pose_publisher ${catkin_LIBRARIES})

add_executable(map_to_img_node src/map_to_img.cpp src/MapAsImageProvider.cpp)

target_link_libraries(map_to_img_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS engix_robot 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS engix_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS robot_pose_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS robot_pose_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

