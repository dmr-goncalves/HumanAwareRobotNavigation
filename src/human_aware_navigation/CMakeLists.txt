cmake_minimum_required(VERSION 2.8.3)
project(human_aware_navigation)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	actionlib
	move_base_msgs
	grid_map_msgs
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
	DetectedStation.msg
	TaskFinished.msg)

## Generate services in the 'srv' folder
 add_service_files(FILES
 	GetPathDistanceDeviation.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include LIBRARIES human_aware_navigation CATKIN_DEPENDS roscpp message_runtime)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(PeopleStationMerger src/PeopleStationMerger_Node.cpp src/PeopleStationMerger.cpp)
add_executable(DistanceMeasurer src/DistanceMeasurer_Node.cpp src/DistanceMeasurer.cpp)
add_executable(GoalSender src/GoalSender_Node.cpp src/GoalSender.cpp)
add_executable(KeyboardManager src/KeyboardManager_Node.cpp src/KeyboardManager.cpp)
add_executable(DetectedPeopleTransformBroadcaster src/DetectedPeopleTransformBroadcaster_Node.cpp src/DetectedPeopleTransformBroadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(PeopleStationMerger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(DistanceMeasurer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(GoalSender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(KeyboardManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(DetectedPeopleTransformBroadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(PeopleStationMerger ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
target_link_libraries(DistanceMeasurer ${catkin_LIBRARIES})
target_link_libraries(GoalSender ${catkin_LIBRARIES})
target_link_libraries(KeyboardManager ${catkin_LIBRARIES})
target_link_libraries(DetectedPeopleTransformBroadcaster ${catkin_LIBRARIES})
