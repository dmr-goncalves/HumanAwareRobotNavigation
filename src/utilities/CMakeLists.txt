cmake_minimum_required(VERSION 2.8.3)
project(utilities)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_library( ${PROJECT_NAME} src/printer.cpp )

target_link_libraries( ${PROJECT_NAME} )
