#include "zero-order.h"
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

namespace librealsense
{
    zero_order::zero_order()
    {
    }
    rs2::frame zero_order::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        return rs2::frame();
    }
    bool zero_order::should_process(const rs2::frame & frame)
    {
        return false;
    }
};