cmake_minimum_required(VERSION 2.8.3)
project(anttweakbar_library)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "64")
else()
  set(ARCH "86")
endif()

if (WIN32)
  set(OPSYS "win")
elseif (UNIX)
  set(OPSYS "linux")
elseif (DARWIN)
  set(OPSYS "osx")
endif ()
message(STATUS "Detected OS: ${OPSYS}, architecture: ${ARCH}")

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES AntTweakBar
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/)

set(AntTweakBar_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${OPSYS}${ARCH}/libAntTweakBar.so.1)

add_library(AntTweakBar SHARED ${AntTweakBar_LIBRARY})
add_custom_command(TARGET AntTweakBar POST_BUILD COMMAND cp ${AntTweakBar_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libAntTweakBar.so)
add_custom_command(TARGET AntTweakBar POST_BUILD COMMAND cp ${AntTweakBar_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libAntTweakBar.so.1)
set_target_properties(AntTweakBar PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(TARGETS AntTweakBar
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

