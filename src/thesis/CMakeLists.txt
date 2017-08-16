cmake_minimum_required(VERSION 2.8.3)
project(thesis)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	grid_map_msgs
	spencer_tracking_msgs
	grid_map_ros
	grid_map_core
	rospy
	message_generation
	image_transport
	cmake_modules
	utilities)
	

find_package(Eigen3 REQUIRED eigen3)
find_package(TinyXML2 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(FILES
	DetectedPeople.msg
	DetectedPerson.msg
	DetectedStations.msg
	DetectedStation.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include LIBRARIES thesis CATKIN_DEPENDS roscpp message_runtime)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(PeopleStationMerger_Node src/PeopleStationMerger_Node.cpp src/PeopleStationMerger.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(PeopleStationMerger_Node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(PeopleStationMerger_Node ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

