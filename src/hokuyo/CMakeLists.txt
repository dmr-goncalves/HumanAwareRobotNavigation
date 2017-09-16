cmake_minimum_required(VERSION 2.8.3)
project(hokuyo)

find_package( catkin REQUIRED COMPONENTS roscpp 
                                         sensor_msgs 
                                         urg_c 
                                         rosconsole 
                                         nodelet 
                                         dynamic_reconfigure 
                                         tf 
                                         laser_proc 
                                         diagnostic_updater 
                                         driver_base
                                         rosconsole
                                         self_test )


find_package( Boost REQUIRED COMPONENTS system thread )

find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()



generate_dynamic_reconfigure_options( cfg/URG.cfg cfg/Hokuyo.cfg )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES urg_c_wrapper libhokuyo
  CATKIN_DEPENDS roscpp sensor_msgs urg_c rosconsole nodelet 
                 dynamic_reconfigure laser_proc
)

###########
## Build ##
###########

include_directories( SYSTEM  ${Boost_INCLUDE_DIRS}  )
include_directories( include ${catkin_INCLUDE_DIRS} )


add_library( urg_c_wrapper src/urg_c_wrapper.cpp    )
add_library( libhokuyo     src/hokuyo.cpp)


add_executable( getID_hokuyo         src/getID_hokuyo.cpp       )
add_executable( getID_urg            src/getID_urg.cpp          )
add_executable( urg_node             src/urg_node.cpp           )
add_executable( hokuyo_node          src/hokuyo_node.cpp        )
add_executable( getFirmwareVersion   src/getFirmwareVersion.cpp )


add_dependencies( urg_node    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                              ${catkin_EXPORTED_TARGETS} )

add_dependencies( hokuyo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                              ${catkin_EXPORTED_TARGETS} )


target_link_libraries( libhokuyo ${Boost_LIBRARIES} ${LOG4CXX_LIBRARY}
                                 ${rosconsole_LIBRARIES} )

target_link_libraries( urg_c_wrapper                     ${catkin_LIBRARIES} )
target_link_libraries( urg_node           urg_c_wrapper  ${catkin_LIBRARIES} )
target_link_libraries( getID_urg          urg_c_wrapper  ${catkin_LIBRARIES} )
target_link_libraries( getID_hokuyo       libhokuyo      ${catkin_LIBRARIES} )
target_link_libraries( hokuyo_node        libhokuyo      ${catkin_LIBRARIES} )
target_link_libraries( getFirmwareVersion libhokuyo      ${catkin_LIBRARIES} )


# ${PROJECT_NAME}_gencfg


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hokuyo_laser hokuyo_laser_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hokuyo_laser.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
