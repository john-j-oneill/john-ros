# Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
# Package Name
project(stephens_bug_algorithms)

# Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES cv_barcode
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(stage_mover src/stage_mover.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(stage_mover
   ${catkin_LIBRARIES}
 )


## Declare a cpp executable
add_executable(bug_nav src/bug_nav.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(bug_nav
   ${catkin_LIBRARIES}
 )

