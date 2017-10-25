# Copyright (c) 2017 Intel Corporation. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

cmake_minimum_required(VERSION 2.8.3)
project(vpx_streamer)

include(ExternalProject)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(PC_VPX REQUIRED vpx)
pkg_check_modules(PC_LIBVA REQUIRED libva)

ExternalProject_Add(webm-tools
  PREFIX third_party/webm-tools
  GIT_REPOSITORY https://github.com/webmproject/webm-tools.git
  GIT_TAG 538dc2d55ea10d5507e1eb353b463bfc34f34b07
  PATCH_COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/webm-tools.patch
  BUILD_COMMAND make -j8
  INSTALL_COMMAND ""
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

set(DEFENSIVE_COMPILE_FLAGS "-std=c++11 -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -fPIE -fPIC -fstack-protector-strong")
set(DEFENSIVE_LINK_FLAGS "-z noexecstack -z relro -z now")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEFENSIVE_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEFENSIVE_COMPILE_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${DEFENSIVE_LINK_FLAGS}")

include_directories(SYSTEM
  ${PC_VPX_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(
  ${source_dir}/shared
  ${source_dir}/../webm-tools-build/libwebm/src/libwebm
  ${source_dir}/../../../libyami/src/libyami-build/include/libyami
)

add_library(${PROJECT_NAME} SHARED
  codec_factory.cpp
  hardware_encoder.cpp
  hardware_decoder.cpp
  software_encoder.cpp
  software_decoder.cpp
  stream_logger.cpp
  stream_muxer.cpp
  stream_parser.cpp
)
add_dependencies(${PROJECT_NAME} webm-tools libyami)
target_link_libraries(${PROJECT_NAME} ${PC_VPX_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${binary_dir}/libwebm-tools.so
                                      ${binary_dir}/../../../libyami/src/libyami-build/lib/libyami.so
)

