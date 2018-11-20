#include "zero-order.h"
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#include <iomanip>
#include "l500/l500.h"

#define METER_TO_MM 1000

namespace librealsense
{
    double get_pixel_rtd(const rs2::vertex v, int baseline)
    {
        auto x = (double)v.x*METER_TO_MM;
        auto y = (double)v.y*METER_TO_MM;
        auto z = (double)v.z*METER_TO_MM;
       
        auto rtd = sqrt(x*x + y*y + z*z) + sqrt((x - baseline) *(x - baseline) + y*y + z*z);
        return v.z ? rtd : 0;
    }

    void z2rtd(const rs2::vertex * vertices, double * rtd, rs2_intrinsics intrinsics, int baseline)
    {
        for (auto i = 0;i < intrinsics.height*intrinsics.width; i++)
        {
            rtd[i] = get_pixel_rtd(vertices[i], baseline);           
        }
    }
    template<typename T>
    T get_zo_point_value(const T * frame_data_in, rs2_intrinsics intrinsics, int zo_point_x, int zo_point_y, int patch_r)
    {
        auto patch_size = (2 * patch_r + 1)*(2 * patch_r + 1);
        std::vector<T> values;

        int i,j;
        for ( i = zo_point_y  -patch_r; i <= (zo_point_y +1+ patch_r ); i ++)
        {
            for ( j = (zo_point_x - patch_r); j <= (zo_point_x +1+ patch_r); j++)
            {
                values.push_back(frame_data_in[i*intrinsics.width + j]);
            }
        }

        std::remove_if(values.begin(), values.end(), [](T val) {return val == 0;});
        std::sort(values.begin(), values.end());
       
        if ((values.size()) % 2 == 0)
        {
            return (values[values.size() / 2 - 1] + values[values.size() / 2 ])/2;
        }
        if (values.size() > 0)
            return values[values.size() / 2 ];
        return 0;
    }

    void detect_zero_order(const double * rtd, const uint16_t * depth_data_in, const uint8_t * ir_data, uint16_t * depth_data_out,
        rs2_intrinsics intrinsics, int rtd_high_threshold, int rtd_low_threshold, int ir_threshold,
        float zo_value, uint8_t iro_value)
    {
        auto ir_dynamic_range = 256;

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
        int zo_point_x,
        int zo_point_y,
        int patch_r,
        int baseline)
    {
        std::vector<double> rtd(intrinsics.height*intrinsics.width);
        z2rtd(vertices, rtd.data(), intrinsics, baseline);
        auto zo_value = get_zo_point_value(rtd.data(), intrinsics, zo_point_x, zo_point_y, patch_r);
        auto iro_value = get_zo_point_value(ir_data, intrinsics, zo_point_x, zo_point_y, patch_r);
        detect_zero_order(rtd.data(), depth_data_in, ir_data, depth_data_out, intrinsics, rtd_high_threshold, rtd_low_threshold, ir_threshold, zo_value, iro_value);
    }

    zero_order::zero_order()
       : _ir_threshold(115),
        _rtd_high_threshold(200),
        _rtd_low_threshold(200),
        _baseline(0),
        _zo_point_x(0),
        _zo_point_y(0),
        _patch_size(5)
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
            400,
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
            400,
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

        auto zo_point_x = std::make_shared<ptr_option<int>>(
            0,
            640,
            1,
            315,
            &_zo_point_x,
            "zero order point x");
        zo_point_x->on_set([zo_point_x](float val)
        {
            if (!zo_point_x->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported zo point x value " << val << " is out of range.");
   
        });

        register_option(RS2_OPTION_FILTER_ZO_POINT_X, zo_point_x);

        auto zo_point_y = std::make_shared<ptr_option<int>>(
            0,
            640,
            1,
            315,
            &_zo_point_y,
            "zero order point y");
        zo_point_x->on_set([zo_point_y](float val)
        {
            if (!zo_point_y->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported zo point y value " << val << " is out of range.");

        });

        register_option(RS2_OPTION_FILTER_ZO_POINT_Y, zo_point_y);

        auto patch_size = std::make_shared<ptr_option<int>>(
            0,
            50,
            1,
            5,
            &_patch_size,
            "patch size");
        zo_point_x->on_set([zo_point_y](float val)
        {
            if (!zo_point_y->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });

        register_option(RS2_OPTION_FILTER_ZO_PATCH_SIZE, patch_size);
    }
    
    bool zero_order::read_zo_point(const rs2::frame & frame)
    {
        if (auto sensor = ((frame_interface*)frame.get())->get_sensor())
        {
            auto dev = const_cast<device_interface*>(&(sensor->get_device()));
            auto debug_dev = dynamic_cast<debug_interface*>(dev);

            if (auto l5 = dynamic_cast<l500_device*>(dev))
            {
                if (l5->read_algo_version() >= 114)
                {
                    l5->read_zo_point(&_zo_point_x, &_zo_point_y);
                    get_option(RS2_OPTION_FILTER_ZO_POINT_X).set(_zo_point_x);
                    get_option(RS2_OPTION_FILTER_ZO_POINT_Y).set(_zo_point_y);
                }
                    
                return true;
            }
        }
        return false;
    }

    bool zero_order::read_baseline(const rs2::frame & frame)
    {
        if (auto sensor = ((frame_interface*)frame.get())->get_sensor())
        {
            auto dev = const_cast<device_interface*>(&(sensor->get_device()));
            auto debug_dev = dynamic_cast<debug_interface*>(dev);

            if (auto l5 = dynamic_cast<l500_device*>(dev))
            {
                if (auto baseline = l5->read_baseline())
                    _baseline = l5->read_baseline();
                return true;
            }
        }
        return false;
    }

    rs2::frame zero_order::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        if (!_zo_point_x || !_zo_point_y)
        {
            if (!read_zo_point(f))
                return f;
        }
        if (!_baseline)
        {
            if (!read_baseline(f))
                _baseline = 31;
        }
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
            _rtd_low_threshold,
            _zo_point_x-1,
            _zo_point_y-1,
            _patch_size,
            _baseline);
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