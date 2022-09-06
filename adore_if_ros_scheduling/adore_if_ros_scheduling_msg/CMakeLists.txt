# ********************************************************************************
# * Copyright (C) 2017-2022 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *   Matthias Nichting
# ********************************************************************************

cmake_minimum_required(VERSION 3.13)
project(adore_if_ros_scheduling_msg)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "adore_if_ros_msg")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")


include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


#target_include_directories(
#  ${PROJECT_NAME}
#  INTERFACE 
#    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
#)

find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
)

add_message_files(
	FILES
	SchedulerNotification.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

find_package(
	catkin REQUIRED COMPONENTS
  	roscpp
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})


include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/LibraryConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)


install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              # "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
        COMPONENT ${PROJECT_NAME}
)
message(WARNING ${PROJECT_SOURCE_DIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/../build/devel/include DESTINATION .)

