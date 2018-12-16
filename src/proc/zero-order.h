// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"
#include "option.h"

namespace librealsense
{
    struct  zero_order_options
    {
        zero_order_options(): 
            ir_threshold(115),
            rtd_high_threshold(200),
            rtd_low_threshold(200),
            baseline(31),
            zo_point_x(315),
            zo_point_y(237),
            patch_size(5),
            z_max(1200),
            ir_min(75),
            threshold_offset(10),
            threshold_scale(20)
        {}

        uint8_t                 ir_threshold;
        uint16_t                rtd_high_threshold;
        uint16_t                rtd_low_threshold;
        int                     baseline;
        bool                    read_baseline;
        int                     zo_point_x;
        int                     zo_point_y;
        int                     patch_size;
        int                     z_max;
        int                     ir_min;
        int                     threshold_offset;
        int                     threshold_scale;
    };

    class zero_order : public generic_processing_block
    {
    public:
        zero_order();
        
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    private:
        bool should_process(const rs2::frame& frame) override;
        rs2::frame prepare_output(const rs2::frame_source& source, rs2::frame input, std::vector<rs2::frame> results) override;
        bool try_read_zo_point(const rs2::frame& frame, int* zo_point_x, int* zo_point_y);
        bool try_read_baseline(const rs2::frame& frame, int* baseline);

        rs2::stream_profile     _source_profile_depth;
        rs2::stream_profile     _target_profile_depth;

        rs2::stream_profile     _source_profile_confidence;
        rs2::stream_profile     _target_profile_confidence;

        rs2::pointcloud         _pc;

        bool                    _first_frame;

        zero_order_options    _options;
    };
}
