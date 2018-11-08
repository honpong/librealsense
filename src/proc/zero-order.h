// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "synthetic-stream.h"

namespace librealsense
{

    class zero_order : public generic_processing_block
    {
    public:
        zero_order();
        rs2::frame process_frame(const rs2::frame_source& source, const rs2::frame& f) override;
    private:
        bool should_process(const rs2::frame& frame) override;

        rs2::stream_profile     _source_profile;
        rs2::stream_profile     _target_profile;

        rs2::pointcloud         _pc;
    };
}
