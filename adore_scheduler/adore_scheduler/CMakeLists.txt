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
project(adore_scheduler)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "adore_if_ros_msg")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")


list(APPEND CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib_adore_scheduling/lib_adore_scheduling/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../adore_if_ros_scheduling/adore_if_ros_scheduling/build/install")


include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)



#target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)



find_package(adore_if_ros_scheduling_msg CONFIG REQUIRED)
find_package(lib_adore_scheduling CONFIG REQUIRED)

find_package(adore_if_ros_scheduling CONFIG REQUIRED)

find_package(catkin REQUIRED 
   COMPONENTS 
      roscpp
)


catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES scheduler
  #CATKIN_DEPENDS 
  #  adore_if_ros_scheduling_msg
)

message(WARNING ${adore_if_ros_scheduling_INCLUDE_DIRS})

add_executable(adore_scheduler_node)
target_sources(adore_scheduler_node PRIVATE src/adore_scheduler_node.cpp)
target_include_directories( 
    adore_scheduler_node 
      PRIVATE include 
        ${catkin_INCLUDE_DIRS}  
        ${CMAKE_PREFIX_PATH}
        ../lib_adore_scheduling/lib_adore_scheduling/build/install/include
        ../adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg/build/install/include
        ${adore_if_ros_scheduling_INCLUDE_DIRS}
)
target_link_libraries(
    adore_scheduler_node 
      PRIVATE 
        ${catkin_LIBRARIES}
	#adore_if_ros_scheduling_msg
  adore_if_ros_scheduling::adore_if_ros_scheduling
)
target_compile_features(adore_scheduler_node PRIVATE cxx_std_17)


install(
  TARGETS
	adore_scheduler_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

