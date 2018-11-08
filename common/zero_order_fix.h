// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once
#include <librealsense2/rs.hpp>

void zero_order_fix(const uint16_t * frame_data_in, uint16_t * frame_data_out,
    rs2_intrinsics intrinsics,
    int zo_point_x = 315, int zo_point_y = 237,
    int rtd_high_threshold = 200, int rtd_low_threshold = -200,
    int baseline = 31, int patch_r =5);

class zero_order_fix_processor : public rs2::processing_block
{
public:

    zero_order_fix_processor();

private:
    rs2::stream_profile     _source_profile;
    rs2::stream_profile     _target_profile;
    
    rs2::pointcloud         _pc;
    rs2::spatial_filter         _hole_filling;
    rs2::hole_filling_filter         _hole_filling2;
    void process_frame(rs2::frameset data, rs2::frame_source& source);
    std::vector<rs2::frame> get_frames(rs2::frameset);
};



