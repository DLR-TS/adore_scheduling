#********************************************************************************
#* Copyright (C) 2017-2023 German Aerospace Center (DLR). 
#* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
#*
#* This program and the accompanying materials are made available under the 
#* terms of the Eclipse Public License 2.0 which is available at
#* http://www.eclipse.org/legal/epl-2.0.
#*
#* SPDX-License-Identifier: EPL-2.0 
#*
#* Contributors: 
#*   Matthias Nichting
#********************************************************************************

cmake_minimum_required(VERSION 3.13)
project(adore_if_ros_scheduling)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "adore_if_ros_scheduling")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")


include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)


list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../libadore_scheduling/libadore_scheduling/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg/build/install")

message(WARNING ${CMAKE_PREFIX_PATH})

#find_package(libadore_scheduling CONFIG REQUIRED)
#find_package(adore_if_ros_scheduling_msg CONFIG REQUIRED)


target_include_directories(
  ${PROJECT_NAME}
  INTERFACE 
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

#target_link_libraries(${PROJECT_NAME}
#  INTERFACE
#    libadore_scheduling
#    adore_if_ros_scheduling_msg
#)


find_package(roscpp REQUIRED)

include(PackagingInstallTargets)

