cmake_minimum_required(VERSION 2.8.3)
project(stations_layer)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
  grid_map_msgs
  grid_map_ros
  grid_map_core
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/StationLayer.cfg
)

catkin_package(
    CATKIN_DEPENDS costmap_2d dynamic_reconfigure
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(stations_layer src/StationLayer.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(stations_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS stations_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
