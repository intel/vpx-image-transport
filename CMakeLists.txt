# Copyright (c) 2016 Intel Corporation. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
cmake_minimum_required(VERSION 2.8.3)
project(vpx_image_transport)

include(ExternalProject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  image_transport
  pluginlib
  message_generation
  std_msgs
  sensor_msgs
  dynamic_reconfigure
  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
pkg_check_modules(PC_VPX REQUIRED vpx)
pkg_check_modules(PC_LIBVA REQUIRED libva)

ExternalProject_Add(webm-tools
  PREFIX third_party/webm-tools
  GIT_REPOSITORY https://github.com/webmproject/webm-tools.git
  GIT_TAG master
  PATCH_COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/webm-tools.patch
  BUILD_COMMAND make -j8
)
ExternalProject_Get_Property(webm-tools binary_dir source_dir)

ExternalProject_Add(libyami
  PREFIX third_party/libyami
  GIT_REPOSITORY https://github.com/01org/libyami.git
  GIT_TAG libyami-1.0.1
  CONFIGURE_COMMAND ${source_dir}/../../../libyami/src/libyami/autogen.sh --prefix=${source_dir}/../../../libyami/src/libyami-build --enable-vp8enc --enable-vp8dec
  BUILD_COMMAND make -j8
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
)

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
#   std_msgs
# )

add_message_files(DIRECTORY msg FILES Packet.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

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
generate_dynamic_reconfigure_options(cfg/VPXPublisher.cfg cfg/VPXSubscriber.cfg)

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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime image_transport pluginlib std_msgs sensor_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM
  ${PC_VPX_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(
  ${source_dir}/shared
  ${source_dir}/../webm-tools-build/libwebm/src/libwebm
  ${source_dir}/../../../libyami/src/libyami-build/include/libyami
)

link_directories(${PC_VPX_LIBRARY_DIRS})
add_definitions(${PC_VPX_CFLAGS_OTHER})

## Declare a C++ library
# add_library(vpx_image_transport
#   src/${PROJECT_NAME}/vpx_image_transport.cpp
# )
add_library(${PROJECT_NAME} src/vpx_publisher.cpp src/vpx_subscriber.cpp src/manifest.cpp src/yami_encoder.cpp src/software_encoder.cpp src/encoder_factory.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(vpx_image_transport ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME} webm-tools libyami)

## Declare a C++ executable
# add_executable(vpx_image_transport_node src/vpx_image_transport_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(vpx_image_transport_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(vpx_image_transport_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${PC_VPX_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${binary_dir}/libwebm-tools.so
                                      ${binary_dir}/../../../libyami/src/libyami-build/lib/libyami.so
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
# install(TARGETS vpx_image_transport vpx_image_transport_node
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
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# add xml file
install(FILES vpx_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vpx_image_transport.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
