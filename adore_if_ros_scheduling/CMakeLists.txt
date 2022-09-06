#********************************************************************************
#* Copyright (C) 2017-2022 German Aerospace Center (DLR). 
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

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "adore_if_ros_msg")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")




include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)


add_subdirectory(adore_if_ros_scheduling_msg "${CMAKE_CURRENT_BINARY_DIR}/adore_if_ros_scheduling_msg")
add_subdirectory(lib_adore_scheduling "${CMAKE_CURRENT_BINARY_DIR}/lib_adore_scheduling")
add_subdirectory(lib_adore_if_ros_scheduling "${CMAKE_CURRENT_BINARY_DIR}/lib_adore_if_ros_scheduling")
add_subdirectory(scheduler "${CMAKE_CURRENT_BINARY_DIR}/scheduler")

