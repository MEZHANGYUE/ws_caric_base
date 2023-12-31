cmake_minimum_required(VERSION 2.8.3)
project(unicon)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mav_msgs
  dynamic_reconfigure
  ## ooqp_eigen_interface
)

find_package(Eigen3 REQUIRED)


include_directories(
	include
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Stop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

generate_dynamic_reconfigure_options(
   cfg/Param.cfg
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unicon
#  CATKIN_DEPENDS eigen geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/unicon.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME} src/unicon_node.cpp)

# set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
