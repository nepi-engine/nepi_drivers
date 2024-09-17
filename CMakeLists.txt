#[[
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
]]
cmake_minimum_required(VERSION 2.8.3)
project(nepi_drivers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Use the verbose version of make
set(CMAKE_VERBOSE_MAKEFILE TRUE)

# Following relies (sometimes, e.g., Jetpack-installed OpenCV) on a symlink from /usr/share/OpenCV to
# true OpenCV cmake folder (e.g., usr/lib/aarch64-linux-gnu/cmake/opencv4 on Jetson)
set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED
    COMPONENTS std_msgs sensor_msgs nepi_edge_sdk_base nepi_ros_interfaces cv_bridge OpenCV  serial pcl_ros pcl_conversions tf2_ros )

find_package(PkgConfig)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
#   std_msgs  # Or other packages containing msgs
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	#INCLUDE_DIRS include
	CATKIN_DEPENDS sensor_msgs nepi_ros_interfaces nepi_edge_sdk_base
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   ptx_drivers/iqr_ros_pan_tilt/include
   ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/ptx_interface.cpp
   src/ptx_node.cpp
)

## IQR Pan/Tilt Node
add_executable(iqr_ros_pan_tilt_node
  ptx_drivers/iqr_ros_pan_tilt/src/iqr_ros_pan_tilt_node.cpp
  ptx_drivers/iqr_ros_pan_tilt/src/modbus_rtu_master.cpp
  ptx_drivers/iqr_ros_pan_tilt/src/PanTiltDriver.cpp  
  ptx_drivers/iqr_ros_pan_tilt/src/QThread.cpp
)

add_dependencies(iqr_ros_pan_tilt_node
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(iqr_ros_pan_tilt_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})


#############
## Install ##
#############



set( DRIVER_INSTALL_FOLDER
  # Must hard-coded 
  /opt/nepi/ros/lib/nepi_drivers
)

catkin_install_python(PROGRAMS
  idx_drivers/idx_genicam_node.py
  idx_drivers/idx_genicam_discovery.py 
  idx_drivers/idx_genicam_driver.py     
  idx_drivers/idx_v4l2_node.py              
  idx_drivers/idx_v4l2_discovery.py
  idx_drivers/idx_v4l2_driver.py
  idx_drivers/idx_zed_node.py
  idx_drivers/idx_onvif_generic_node.py 
  idx_drivers/idx_onvif_econroutecam_node.py
  idx_drivers/idx_onvif_econroutecam_driver.py
  idx_drivers/idx_onvif_generic_driver.py
  DESTINATION ${DRIVER_INSTALL_FOLDER}
)


catkin_install_python(PROGRAMS
  ptx_drivers/ptx_iqr_discovery.py
  ptx_drivers/ptx_iqr_node.py
  ptx_drivers/ptx_onvif_generic_driver.py
  ptx_drivers/ptx_onvif_generic_node.py 
  ptx_drivers/ptx_onvif_sidusss109ht_driver.py 
  ptx_drivers/ptx_onvif_sidusss109ht_node.py
  DESTINATION ${DRIVER_INSTALL_FOLDER}
)


catkin_install_python(PROGRAMS
  lsx_drivers/lsx_aftowerlight_node.py
  lsx_drivers/lsx_aftowerlight_discovery.py
  lsx_drivers/lsx_sealite_node.py
  lsx_drivers/lsx_sealite_discovery.py
  DESTINATION ${DRIVER_INSTALL_FOLDER}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  iqr_ros_pan_tilt_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${DRIVER_INSTALL_FOLDER}
)

## Mark cpp header files for installation - Msgs and Services (generated headers) are under the project name subdir
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Non-generated headers are located directly in include and get copied to the root of include in the install directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nepi_edge_sdk_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
