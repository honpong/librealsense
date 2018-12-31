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
    std::vector <T> get_zo_point_values(const T * frame_data_in, rs2_intrinsics intrinsics, int zo_point_x, int zo_point_y, int patch_r)
    {
        std::vector<T> values;
        for (auto i = zo_point_y - 1 - patch_r; i <= (zo_point_y + patch_r) && i < intrinsics.height; i++)
        {
            for (auto j = (zo_point_x - 1 - patch_r); j <= (zo_point_x + patch_r) && i < intrinsics.width; j++)
            {
                values.push_back(frame_data_in[i*intrinsics.width + j]);
            }
        }
        

        return values;
    }

    template<typename T>
    T get_zo_point_value(std::vector <T> values)
    {
        std::sort(values.begin(), values.end());

        if ((values.size()) % 2 == 0)
        {
            return (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2;
        }
        else if (values.size() > 0)
            return values[values.size() / 2];

        return 0;
    }

    bool try_get_zo_rtd_ir_point_values(const double * rtd, const uint16_t * depth_data_in, const uint8_t * ir_data, 
        rs2_intrinsics intrinsics, const zero_order_options& options,
        double *rtd_zo_value, uint8_t* ir_zo_data)
    {
        auto values_rtd = get_zo_point_values(rtd, intrinsics, options.zo_point_x, options.zo_point_y, options.patch_size);
        auto values_ir = get_zo_point_values(ir_data, intrinsics, options.zo_point_x, options.zo_point_y, options.patch_size);
        auto values_z = get_zo_point_values(depth_data_in, intrinsics, options.zo_point_x, options.zo_point_y, options.patch_size);

          for (auto i = 0;i < values_rtd.size();i++)
        {
              if ((double)values_z[i]/ (double)8 > options.z_max || values_ir[i] < options.ir_min)
              {
                  values_rtd[i] = 0;  
                  values_ir[i] = 0;
              }       
        }
        
        
        values_rtd.erase(std::remove_if(values_rtd.begin(), values_rtd.end(), [](double val)
        {
            return val == 0;

        }), values_rtd.end());

        values_ir.erase(std::remove_if(values_ir.begin(), values_ir.end(), [](uint8_t val)
        {
            return val == 0;

        }), values_ir.end());

        if (values_rtd.size() == 0 || values_rtd.size() == 0)
            return false;

        *rtd_zo_value = get_zo_point_value(values_rtd);
        *ir_zo_data = get_zo_point_value(values_ir);

        return true;

    }

    void detect_zero_order(const double * rtd, const uint16_t * depth_data_in, const uint8_t * ir_data, std::function<void(int index, bool zero)> zero_pixel,
        rs2_intrinsics intrinsics, const zero_order_options& options,
       float zo_value, uint8_t iro_value)
    {
        int ir_dynamic_range = 256;
        
        //auto i_threshold_relative = ((float)ir_threshold / (float)(std::pow(ir_dynamic_range , 2)))*(std::pow((iro_value) , 2));

        auto r = (double)std::exp(((double)ir_dynamic_range / 2 + options.threshold_offset - iro_value) / (double)options.threshold_scale);

        auto res = (1 + r);
        auto i_threshold_relative = (double)options.ir_threshold / res;
        for (auto i = 0; i < intrinsics.height*intrinsics.width; i++)
        {
            auto rtd_val = rtd[i];
            auto ir_val = ir_data[i];

            if (depth_data_in[i] > 0 && ir_val < i_threshold_relative && rtd_val >(zo_value - options.rtd_low_threshold) && rtd_val < (zo_value + options.rtd_high_threshold))
            {
                zero_pixel(i, true);
            }
            else
            {
                zero_pixel(i, false);
            }
        }
    }

    bool zero_order_fix(const uint16_t * depth_data_in, const uint8_t * ir_data, std::function<void(int index, bool zero)> zero_pixel,
        const rs2::vertex* vertices,
        rs2_intrinsics intrinsics,
        const zero_order_options& options)
    {
        std::vector<double> rtd(intrinsics.height*intrinsics.width);
        z2rtd(vertices, rtd.data(), intrinsics, options.baseline);
        double rtd_zo_value;
        uint8_t ir_zo_value;

        if (try_get_zo_rtd_ir_point_values(rtd.data(), depth_data_in, ir_data, intrinsics, 
            options, &rtd_zo_value, &ir_zo_value))
        {
            detect_zero_order(rtd.data(), depth_data_in, ir_data, zero_pixel, intrinsics,
                options, rtd_zo_value, ir_zo_value);
            return true;
        }
        return false;
    }

    zero_order::zero_order()
       : _first_frame(true)
    {
        auto ir_threshold = std::make_shared<ptr_option<uint8_t>>(
            0,
            255,
            1,
            115,
            &_options.ir_threshold,
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
            &_options.rtd_high_threshold,
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
            &_options.rtd_low_threshold,
            "rtd high threshold");
        rtd_low_threshold->on_set([rtd_low_threshold](float val)
        {
            if (!rtd_low_threshold->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported rtd low threshold " << val << " is out of range.");

        });

        register_option(RS2_OPTION_FILTER_ZO_RTD_LOW_THRESHOLD, rtd_low_threshold);

      

        auto baseline = std::make_shared<ptr_option<int>>(
            0,
            50,
            1,
            31,
            &_options.baseline,
            "baseline");
        baseline->on_set([baseline](float val)
        {
            if (!baseline->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });
        register_option(RS2_OPTION_FILTER_ZO_BASELINE, baseline);
    
        auto patch_size = std::make_shared<ptr_option<int>>(
            0,
            50,
            1,
            5,
            &_options.patch_size,
            "patch size");
        patch_size->on_set([patch_size](float val)
        {
            if (!patch_size->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });
        register_option(RS2_OPTION_FILTER_ZO_PATCH_SIZE, patch_size);

        auto zo_max = std::make_shared<ptr_option<int>>(
            0,
            65535,
            1,
            1200,
            &_options.z_max,
            "zo max value");
        zo_max->on_set([zo_max](float val)
        {
            if (!zo_max->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });
        register_option(RS2_OPTION_FILTER_ZO_MAX_VALUE, zo_max);

        auto ir_min = std::make_shared<ptr_option<int>>(
            0,
            256,
            1,
            75,
            &_options.ir_min,
            "minimum IR value (saturation)");
        ir_min->on_set([ir_min](float val)
        {
            if (!ir_min->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });
        register_option(RS2_OPTION_FILTER_ZO_IR_MIN_VALUE, ir_min);
       
        auto offset = std::make_shared<ptr_option<int>>(
            0,
            1000,
            1,
            10,
            &_options.threshold_offset,
            "minimum IR value (saturation)");
        offset->on_set([offset](float val)
        {
            if (!offset->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });
        register_option(RS2_OPTION_FILTER_ZO_THRESHOLD_OFFSET, offset);
        
        auto scale = std::make_shared<ptr_option<int>>(
            0,
            2000,
            1,
            20,
            &_options.threshold_scale,
            "minimum IR value (saturation)");
        scale->on_set([scale](float val)
        {
            if (!scale->is_valid(val))
                throw invalid_value_exception(to_string()
                    << "Unsupported patch size value " << val << " is out of range.");

        });
        register_option(RS2_OPTION_FILTER_ZO_THRESHOLD_SCALE, scale);

    }
    
    bool zero_order::try_read_zo_point(const rs2::frame & frame, int* zo_point_x, int* zo_point_y)
    {
        if (auto sensor = ((frame_interface*)frame.get())->get_sensor())
        {
            auto dev = const_cast<device_interface*>(&(sensor->get_device()));
            auto debug_dev = dynamic_cast<debug_interface*>(dev);

            int ver = 0;
            if (auto l5 = dynamic_cast<l500_device*>(dev))
            {
                if (ver = l5->read_algo_version() >= 115)
                {
                    l5->read_zo_point(zo_point_x, zo_point_y);
                    return true;
                } 
                else
                {
                    LOG_WARNING("Could not read zo point values the algo version is " << ver);
                }
            }
        }
        return false;
    }

    bool zero_order::try_read_baseline(const rs2::frame & frame, int* baseline)
    {
        if (auto sensor = ((frame_interface*)frame.get())->get_sensor())
        {
            auto dev = const_cast<device_interface*>(&(sensor->get_device()));
            auto debug_dev = dynamic_cast<debug_interface*>(dev);

            if (auto l5 = dynamic_cast<l500_device*>(dev))
            {
                if (*baseline = l5->read_baseline())
                    return true;
            }
        }
        return false;
    }

    rs2::frame zero_order::process_frame(const rs2::frame_source & source, const rs2::frame & f)
    {
        std::vector<rs2::frame> result;

        auto zo_point_x_def = 0;
        auto zo_point_y_def = 0;

        if (_first_frame)
        {           
            if (try_read_zo_point(f, &zo_point_x_def, &zo_point_y_def))
            {
                _options.zo_point_x = zo_point_x_def;
                _options.zo_point_y = zo_point_y_def;
            }

            auto zo_point_x = std::make_shared<ptr_option<int>>(
                20,
                f.get_profile().as< rs2::video_stream_profile>().width(),
                1,
                zo_point_x_def,
                &_options.zo_point_x,
                "zero order point x");
            zo_point_x->on_set([zo_point_x](float val)
            {
                if (!zo_point_x->is_valid(val))
                    throw invalid_value_exception(to_string()
                        << "Unsupported zo point x value " << val << " is out of range.");

            });

            register_option(RS2_OPTION_FILTER_ZO_POINT_X, zo_point_x);

            auto zo_point_y = std::make_shared<ptr_option<int>>(
                20,
                f.get_profile().as< rs2::video_stream_profile>().height(),
                1,
                zo_point_y_def,
                &_options.zo_point_y,
                "zero order point y");
            zo_point_y->on_set([zo_point_y](float val)
            {
                if (!zo_point_y->is_valid(val))
                    throw invalid_value_exception(to_string()
                        << "Unsupported zo point y value " << val << " is out of range.");

            });

            register_option(RS2_OPTION_FILTER_ZO_POINT_Y, zo_point_y);

            int baseline;
            try_read_baseline(f, &baseline);
            _first_frame = false;
        }
     
        auto data = f.as<rs2::frameset>();
        
       //auto start = std::chrono::high_resolution_clock::now();
        if (!_source_profile_depth)
            _source_profile_depth = data.get_depth_frame().get_profile();

        if (!_target_profile_depth)
            _target_profile_depth = _source_profile_depth.clone(_source_profile_depth.stream_type(), _source_profile_depth.stream_index(), _source_profile_depth.format());

        auto depth_frame = data.get_depth_frame();
        auto ir_frame = data.get_infrared_frame();
        auto confidence_frame = data.first_or_default(RS2_STREAM_CONFIDENCE);

        auto points = _pc.calculate(depth_frame);

        auto depth_out = source.allocate_video_frame(_target_profile_depth, depth_frame, 0, 0, 0, 0, RS2_EXTENSION_DEPTH_FRAME);

        rs2::frame confidence_out;
        if (confidence_frame)
        {
            if(!_source_profile_confidence)
                _source_profile_confidence = confidence_frame.get_profile();

            if (!_target_profile_confidence)
                _target_profile_confidence = _source_profile_confidence.clone(_source_profile_confidence.stream_type(), _source_profile_confidence.stream_index(), _source_profile_confidence.format());

            confidence_out = source.allocate_video_frame(_source_profile_confidence, confidence_frame, 0, 0, 0, 0, RS2_EXTENSION_VIDEO_FRAME);
        }
        auto depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();


        if (zero_order_fix((const uint16_t*)depth_frame.get_data(),
            (const uint8_t*)ir_frame.get_data(),
            [&](int index, bool zero) 
        {
            ((uint16_t*)depth_out.get_data())[index] = zero ? 0 : ((uint16_t *)depth_frame.get_data())[index];

            if (confidence_frame)
            {
                ((uint8_t*)confidence_out.get_data())[index] = zero ? 0 : ((uint8_t *)confidence_frame.get_data())[index];
            }
        },
            points.get_vertices(),
            depth_intrinsics,
            _options))
        {
            //auto end = std::chrono::high_resolution_clock::now();
            //auto diff = std::chrono::duration<double, std::milli>((end - start));
            //std::cout << diff.count() << std::endl;
            result.push_back(depth_out);
            result.push_back(ir_frame);
            if (confidence_frame)
                result.push_back(confidence_out);

        }
            
        else
        {
            result.push_back(depth_frame);
            result.push_back(ir_frame);
            if (confidence_frame)
                result.push_back(confidence_frame);
        }
        return source.allocate_composite_frame(result);
    }

    bool zero_order::should_process(const rs2::frame& frame)
    {
        if (auto set = frame.as<rs2::frameset>())
        {
            if (!set.get_depth_frame() || !set.get_infrared_frame())
            {
                return false;
            }
            if (!_first_frame && (_options.zo_point_x == 0 || _options.zo_point_y == 0))
                return false;

            return true;
        }
        return false;
    }

    rs2::frame zero_order::prepare_output(const rs2::frame_source & source, rs2::frame input, std::vector<rs2::frame> results)
    {
        if (auto composite = input.as<rs2::frameset>())
        {
            composite.foreach([&](rs2::frame f) 
            {
                if (f.get_profile().stream_type() != RS2_STREAM_DEPTH && f.get_profile().stream_type() != RS2_STREAM_INFRARED && results.size()>0)
                    results.push_back(f);
            });
        }
        return source.allocate_composite_frame(results);
    }
   
};
