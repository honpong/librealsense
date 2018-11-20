// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"
#include "option.h"

namespace librealsense
{

    class zero_order : public generic_processing_block
    {
    public:
        zero_order();
        
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    private:
        bool should_process(const rs2::frame& frame) override;
        bool read_zo_point(const rs2::frame& frame);
        bool read_baseline(const rs2::frame& frame);

        rs2::stream_profile     _source_profile;
        rs2::stream_profile     _target_profile;
        rs2::pointcloud         _pc;

        uint8_t                 _ir_threshold;
        uint16_t                _rtd_high_threshold;
        uint16_t                _rtd_low_threshold;
        float                   _baseline;
        int                     _zo_point_x;
        int                     _zo_point_y;
        int                     _patch_size;
    };
}
