# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesTm2CalibrationWrite)

if(BUILD_EXAMPLES)
    add_executable(rs-tm2-calibration-write rs-tm2-calibration-write.cpp)
    set_property(TARGET rs-tm2-calibration-write PROPERTY CXX_STANDARD 11)
    target_link_libraries(rs-tm2-calibration-write ${DEPENDENCIES})
    set_target_properties (rs-tm2-calibration-write PROPERTIES FOLDER Examples)
    install(TARGETS rs-tm2-calibration-write RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
