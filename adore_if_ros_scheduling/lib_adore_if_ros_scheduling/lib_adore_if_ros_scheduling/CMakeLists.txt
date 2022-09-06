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
project(lib_adore_if_ros_scheduling)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "adore_if_ros_msg")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")


include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib_adore_scheduling/lib_adore_scheduling/build/install")

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE 
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

find_package(lib_adore_scheduling CONFIG REQUIRED)
find_package(roscpp REQUIRED)

#target_link_libraries(${PROJECT_NAME}
#  PUBLIC
#)

#target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(TARGETS ${PROJECT_NAME} 
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/LibraryConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)


install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE lib_adore_if_ros_scheduling::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              # "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
        COMPONENT ${PROJECT_NAME}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${PROJECT_SOURCE_DIR}/build/devel) 

