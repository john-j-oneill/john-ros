cmake_minimum_required(VERSION 2.8.3)
project(raspicam_stills)

# Only build on ARM, since this only works on the Pi anyways.
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*) 

  set(CMAKE_CXX_FLAGS "-fpermissive")

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    tf
  )

  ## System dependencies are found with CMake's conventions
  # find_package(Boost REQUIRED COMPONENTS system)


  ## Uncomment this if the package has a setup.py. This macro ensures
  ## modules and global scripts declared therein get installed
  ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
  # catkin_python_setup()

  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################

  ## To declare and build messages, services or actions from within this
  ## package, follow these steps:
  ## * Let MSG_DEP_SET be the set of packages whose message types you use in
  ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
  ## * In the file package.xml:
  ##   * add a build_depend tag for "message_generation"
  ##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
  ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
  ##     but can be declared for certainty nonetheless:
  ##     * add a run_depend tag for "message_runtime"
  ## * In this file (CMakeLists.txt):
  ##   * add "message_generation" and every package in MSG_DEP_SET to
  ##     find_package(catkin REQUIRED COMPONENTS ...)
  ##   * add "message_runtime" and every package in MSG_DEP_SET to
  ##     catkin_package(CATKIN_DEPENDS ...)
  ##   * uncomment the add_*_files sections below as needed
  ##     and list every .msg/.srv/.action file to be processed
  ##   * uncomment the generate_messages entry below
  ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

  ## Generate messages in the 'msg' folder
  # add_message_files(
  #   FILES
  #   Message1.msg
  #   Message2.msg
  # )

  ## Generate services in the 'srv' folder
  # add_service_files(
  #   FILES
  #   Service1.srv
  #   Service2.srv
  # )

  ## Generate actions in the 'action' folder
  # add_action_files(
  #   FILES
  #   Action1.action
  #   Action2.action
  # )

  ## Generate added messages and services with any dependencies listed here
  # generate_messages(
  #   DEPENDENCIES
  #   sensor_msgs
  # )

  ################################################
  ## Declare ROS dynamic reconfigure parameters ##
  ################################################

  ## To declare and build dynamic reconfigure parameters within this
  ## package, follow these steps:
  ## * In the file package.xml:
  ##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
  ## * In this file (CMakeLists.txt):
  ##   * add "dynamic_reconfigure" to
  ##     find_package(catkin REQUIRED COMPONENTS ...)
  ##   * uncomment the "generate_dynamic_reconfigure_options" section below
  ##     and list every .cfg file to be processed

  ## Generate dynamic reconfigure parameters in the 'cfg' folder
  # generate_dynamic_reconfigure_options(
  #   cfg/DynReconf1.cfg
  #   cfg/DynReconf2.cfg
  # )

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
  #  LIBRARIES raspicam_stills
  #  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs tf
  #  DEPENDS system_lib
  )


  ## System dependencies are found with CMake's conventions
  # find_package(Boost REQUIRED COMPONENTS system)

  FIND_LIBRARY(mmal_LIBS NAMES mmal PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalcore_LIBS NAMES mmal_core PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalutil_LIBS NAMES mmal_util PATHS /opt/vc/lib)
  FIND_LIBRARY(mmalvcclient_LIBS NAMES mmal_vc_client PATHS /opt/vc/lib)
  IF( (NOT mmal_LIBS ) OR (NOT mmalutil_LIBS) OR (NOT mmalcore_LIBS) OR (NOT mmalvcclient_LIBS) )
      MESSAGE(FATAL_ERROR "Could not find mmal libraries")
  ENDIF()

  FIND_LIBRARY(vcos_LIBS NAMES vcos PATHS /opt/vc/lib)
  IF( (NOT vcos_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find vcos library")
  ENDIF()

  FIND_LIBRARY(bcmhost_LIBS NAMES bcm_host PATHS /opt/vc/lib)
  IF( (NOT bcmhost_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find bcm_host library")
  ENDIF()

  FIND_LIBRARY(vchostif_LIBS NAMES vchostif PATHS /opt/vc/lib)
  IF( (NOT vchostif_LIBS ))
      MESSAGE(FATAL_ERROR "Could not find vchostif library")
  ENDIF()

  set(RPI_LIBRARIES ${RPI_LIBRARIES} 
  ${bcmhost_LIBS}
  ${vcos_LIBS} 
  ${mmal_LIBS} 
  ${mmalcore_LIBS} 
  ${mmalutil_LIBS}  
  ${mmalvcclient_LIBS} 
  ${vchostif_LIBS} 
  )

  ###########
  ## Build ##
  ###########

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  # include_directories(include)
  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    src/
    /home/john/userland
    /opt/vc/include
    /opt/vc/include/interface/vcos/pthreads
    /opt/vc/include/interface/vmcs_host/linux
  )

  link_directories(
    src/
  )

  ## Declare a C++ library
  # add_library(raspicam_stills
  #   src/${PROJECT_NAME}/raspicam_stills.cpp
  # )

  ## Add cmake target dependencies of the library
  ## as an example, code may need to be generated before libraries
  ## either from message generation or dynamic reconfigure
  # add_dependencies(raspicam_stills ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


  ## Declare a cpp library
   add_library(RaspiCamControl STATIC
     src/RaspiCamControl.c
   )

  ## Declare a cpp library
   add_library(RaspiCLI STATIC
     src/RaspiCLI.c 
   )

  ## Declare a cpp library
   add_library(RaspiPreview STATIC
     src/RaspiPreview.c 
   )

  ## Declare a C++ executable
  add_executable(raspicam_stills_node
    src/raspicam_stills_node.cpp
  )

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  # add_dependencies(raspicam_stills_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  set (MMAL_LIBS mmal_core mmal_util mmal_vc_client)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(raspicam_stills_node
    ${catkin_LIBRARIES}
    ${RPI_LIBRARIES} 
    RaspiCamControl
    RaspiCLI
    RaspiPreview
  )

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
  # install(TARGETS raspicam_stills raspicam_stills_node
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
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_raspicam_stills.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)

ENDIF()
