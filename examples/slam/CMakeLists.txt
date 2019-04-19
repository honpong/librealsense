# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(rs-slam)

if(BUILD_EXAMPLES)
    add_executable(${PROJECT_NAME} rs-slam.cpp)
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
    target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
    set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER Examples)
    install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
