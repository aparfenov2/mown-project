cmake_minimum_required(VERSION 3.5.1)

project(ddrnet)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(
  CATKIN_PACKAGE_DEPENDENCIES 
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
# set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 4.1.0 REQUIRED COMPONENTS core imgproc )
include_directories(${OpenCV_INCLUDE_DIRS})


# find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

# include_directories(/home/midas/TensorRT-7.0.0.11/include/)
# link_directories(/home/midas/TensorRT-7.0.0.11/lib/)

## Declare a cpp library
# add_library(${PROJECT_NAME}
#   src/GridMapCvProcessing.cpp
#   src/InpaintFilter.cpp
# )

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

add_executable(${PROJECT_NAME} 
  src/ddrnet_node.cpp
  src/ddrnet_hr.cpp
  )
target_link_libraries(${PROJECT_NAME} nvinfer)
target_link_libraries(${PROJECT_NAME} cudart)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_tool
  src/ddrnet_tool.cpp
  src/ddrnet_hr.cpp
  )
target_link_libraries(${PROJECT_NAME}_tool nvinfer)
target_link_libraries(${PROJECT_NAME}_tool cudart)
target_link_libraries(${PROJECT_NAME}_tool ${OpenCV_LIBS})

# add_definitions(-O2 -pthread)
# add_dependencies(${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation.
# install(
#   FILES filter_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

