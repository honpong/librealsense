# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesPoseMap)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-pose-change-origin rs-pose-change-origin.cpp ../example.hpp)
    target_include_directories(rs-pose-change-origin PUBLIC ../)
    include_directories(rs-pose-change-origin)
    target_link_libraries(rs-pose-change-origin ${DEPENDENCIES})
    set_target_properties (rs-pose-change-origin PROPERTIES FOLDER Examples)
    install(TARGETS rs-pose-change-origin RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
