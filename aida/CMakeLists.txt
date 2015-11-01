cmake_minimum_required(VERSION 2.8.3)
project(aida)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation tf pcl_ros roslib)
find_package(PCL 1.6)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LineMsg.msg
  Walls.msg
  Human.msg
  People.msg
  Velocity.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetDirection.srv
  Update.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES aida
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ../ppl_detection/msg_gen/cpp/include
)

## Declare a cpp library
# add_library(aida
#   src/${PROJECT_NAME}/aida.cpp
# )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
# add_executable(aida_node src/aida_node.cpp)

add_executable(switch src/switch.cpp)
target_link_libraries(switch ${catkin_LIBRARIES})

add_executable(la src/la.cpp)
target_link_libraries(la ${catkin_LIBRARIES})

add_executable (detector src/detector.cpp)
target_link_libraries (detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(turn src/turn.cpp)
target_link_libraries(turn ${catkin_LIBRARIES})

add_executable(hallway src/hallway.cpp)
target_link_libraries(hallway ${catkin_LIBRARIES})

add_executable(end_of_hallway src/end_of_hallway.cpp)
target_link_libraries(end_of_hallway ${catkin_LIBRARIES})

add_executable(reset src/reset.cpp)
target_link_libraries(reset ${catkin_LIBRARIES})

add_executable(lis src/lis.cpp)
target_link_libraries(lis ${catkin_LIBRARIES})

add_executable(bestFit src/bestFit.cpp)
target_link_libraries(bestFit ${catkin_LIBRARIES})

## Add dependencies to the executable
# add_dependencies(aida_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
# target_link_libraries(aida_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS aida aida_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aida.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
