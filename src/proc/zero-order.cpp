#include "zero-order.h"
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#define METER_TO_MM 1000
namespace librealsense
{
    float get_pixel_rtd(const rs2::vertex v, int baseline)
    {
        auto x = v.x*METER_TO_MM;
        auto y = v.y*METER_TO_MM;
        auto z = v.z*METER_TO_MM;

        auto rtd = sqrt(x*x + y*y + z*z) + sqrt((x - baseline) *(x - baseline) + y*y + z*z);
        return v.z ? rtd : 0;
    }

    void z2rtd(const rs2::vertex * vertices, float * rtd, rs2_intrinsics intrinsics, int baseline)
    {
        std::ifstream ifile;
        std::vector<uint16_t> rtd_(640 * 480);
        ifile.open("sample_4out_rtd.640x480.bin16", std::ios::binary);
        ifile.read((char*)rtd_.data(), 640 * 480 * 2);

        for (auto i = 0;i < intrinsics.height*intrinsics.width; i++)
        {
            rtd[i] = get_pixel_rtd(vertices[i], baseline);

            if (rtd[i] != rtd_[i])
            {
                std::cout << "";
            }
        }

    }
    template<typename T>
    T get_zo_point_value(const T * frame_data_in, rs2_intrinsics intrinsics, int zo_point_x, int zo_point_y, int patch_r)
    {
        auto patch_size = (2 * patch_r + 1)*(2 * patch_r + 1);
        std::vector<T> values(patch_size);

        for (auto i = (zo_point_y - patch_r)* intrinsics.width; i < (zo_point_y + patch_r + 1)* intrinsics.width; i += intrinsics.width)
        {
            for (auto j = (zo_point_x - patch_r); j < (zo_point_x + patch_r + 1); j++)
            {
                values.push_back(frame_data_in[i + j]);
            }
        }

        std::remove_if(values.begin(), values.end(), [](T val) {return val == 0;});
        std::sort(values.begin(), values.end());

        if (values.size() > 0)
            return values[values.size() / 2 - 1];
        return 0;
    }

    void detect_zero_order(const float * rtd, const uint16_t * depth_data_in, const uint8_t * ir_data, uint16_t * depth_data_out,
        rs2_intrinsics intrinsics, int rtd_high_threshold, int rtd_low_threshold, int ir_threshold,
        uint16_t zo_value, uint8_t iro_value)
    {
        auto ir_dynamic_range = 256;
        //auto i_threshold_relative = (ir_threshold*((iro_value) ^ 2)) / (ir_dynamic_range ^ 2);
        //auto ref = ((float)ir_threshold / (float)(ir_dynamic_range ^ 2));
        auto i_threshold_relative = ((float)ir_threshold / (float)(std::pow(ir_dynamic_range , 2)))*(std::pow((iro_value) , 2));
        for (auto i = 0; i < intrinsics.height*intrinsics.width; i++)
        {
            auto rtd_val = rtd[i];
            auto ir_val = ir_data[i];

            if (depth_data_in[i] > 0 && ir_val < i_threshold_relative && rtd_val >(zo_value - rtd_low_threshold) && rtd_val < (zo_value + rtd_high_threshold))
            {
                depth_data_out[i] = 0;
            }
            else
            {
                depth_data_out[i] = depth_data_in[i];
            }
        }
    }

    void zero_order_fix(const uint16_t * depth_data_in, const uint8_t * ir_data, uint16_t * depth_data_out,
        const rs2::vertex* vertices,
        rs2_intrinsics intrinsics,
        int ir_threshold,
        int rtd_high_threshold, 
        int rtd_low_threshold,
        int patch_r = 5,
        int baseline = 31,
        int zo_point_x = 315, 
        int zo_point_y = 237)
    {

        std::vector<float> rtd(intrinsics.height*intrinsics.width);
        z2rtd(vertices, rtd.data(), intrinsics, baseline);
        auto zo_value = get_zo_point_value(rtd.data(), intrinsics, zo_point_x, zo_point_y, patch_r);
        auto iro_value = get_zo_point_value(ir_data, intrinsics, zo_point_x, zo_point_y, patch_r);
        detect_zero_order(rtd.data(), depth_data_in, ir_data, depth_data_out, intrinsics, rtd_high_threshold, rtd_low_threshold, ir_threshold, zo_value, iro_value);
    }

    zero_order::zero_order()
       : _ir_threshold(115),
        _rtd_high_threshold(200),
        _rtd_low_threshold(200)
    {
        auto ir_threshold = std::make_shared<ptr_option<uint8_t>>(
            0,
            255,
            1,
            115,
            &_ir_threshold,
            "ir threshold");
        ir_threshold->on_set([ir_threshold](float val)
        {
            if (!ir_threshold->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported ir threshold " << val << " is out of range.");

        });

        register_option(RS2_OPTION_FILTER_ZO_IR_THRESHOLD, ir_threshold);

        auto rtd_high_threshold = std::make_shared<ptr_option<uint16_t>>(
            0,
            std::numeric_limits<uint16_t>::max(),
            1,
            200,
            &_rtd_high_threshold,
            "rtd high threshold");
        rtd_high_threshold->on_set([rtd_high_threshold](float val)
        {
            if (!rtd_high_threshold->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported rtd high threshold " << val << " is out of range.");

        });

        register_option(RS2_OPTION_FILTER_ZO_RTD_HIGH_THRESHOLD, rtd_high_threshold);

        auto rtd_low_threshold = std::make_shared<ptr_option<uint16_t>>(
            0,
            std::numeric_limits<uint16_t>::max(),
            1,
            200,
            &_rtd_low_threshold,
            "rtd high threshold");
        rtd_low_threshold->on_set([rtd_low_threshold](float val)
        {
            if (!rtd_low_threshold->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported rtd low threshold " << val << " is out of range.");

        });

        register_option(RS2_OPTION_FILTER_ZO_RTD_LOW_THRESHOLD, rtd_low_threshold);
    }

    rs2::frame zero_order::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        auto data = f.as<rs2::frameset>();
        
        //auto start = std::chrono::high_resolution_clock::now();
        if (!_source_profile)
            _source_profile = data.get_depth_frame().get_profile();

        if (!_target_profile)
            _target_profile = _source_profile.clone(_source_profile.stream_type(), _source_profile.stream_index(), _source_profile.format());

        auto depth_frame = data.get_depth_frame();
        auto ir_frame = data.get_infrared_frame();

        auto points = _pc.calculate(depth_frame);

        auto out_frame = source.allocate_video_frame(_target_profile, depth_frame, 0, 0, 0, 0, RS2_EXTENSION_DEPTH_FRAME);
        auto depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        zero_order_fix((const uint16_t*)depth_frame.get_data(), 
            (const uint8_t*)ir_frame.get_data(), 
            (uint16_t*)out_frame.get_data(), 
            points.get_vertices(), 
            depth_intrinsics,
            _ir_threshold,
            _rtd_high_threshold,
            _rtd_low_threshold);
        //auto end = std::chrono::high_resolution_clock::now();
        //auto diff = std::chrono::duration<double, std::milli>((end - start));
        //std::cout << diff.count() << std::endl;
        return out_frame;
    }
    bool zero_order::should_process(const rs2::frame& frame)
    {
        if (auto set = frame.as<rs2::frameset>())
        {
            if (!set.get_depth_frame() || !set.get_infrared_frame())
                return false;
            return true;
        }
        return false;
    }
};