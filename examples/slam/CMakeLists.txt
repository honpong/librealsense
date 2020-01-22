# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(rs-slam)

add_subdirectory(tracker/src)

find_package(tracker QUIET)
if (NOT TARGET tracker)
    return()
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(${PROJECT_NAME} rs-slam.cpp rs-slam-trajectory.hpp)
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
    target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES} tracker)
    include_directories(../ ../../common/res ${LZ4_DIR})
    set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER Examples)
    install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/tracker/cmake)
    find_package(Eigen3 REQUIRED)
    add_executable(rs-large-area-mapping rs-large-area-mapping.cpp rs-slam-trajectory.hpp tracker/src/tracker/adjustment_layer.h)
    set_property(TARGET rs-large-area-mapping PROPERTY CXX_STANDARD 11)
    target_compile_definitions(rs-large-area-mapping PUBLIC _SILENCE_CXX17_ADAPTOR_TYPEDEFS_DEPRECATION_WARNING) # Eigen
    target_link_libraries(rs-large-area-mapping ${DEPENDENCIES} Eigen3::Eigen)
    include_directories(../ ../../common/res ${LZ4_DIR} tracker/src/tracker)
    set_target_properties (rs-large-area-mapping PROPERTIES FOLDER Examples)
    install(TARGETS rs-large-area-mapping RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
