#include "zero_order_fix.h"
#include "../include/librealsense2/rsutil.h"
#include "../include/librealsense2/hpp/rs_frame.hpp"
#include "math.h"
#include <fstream>

uint16_t get_pixel_rtd(const rs2::vertex v, int baseline)
{
    auto rtd = sqrt(v.x*v.x + v.y*v.y + v.z*v.z) + sqrt((v.x - baseline) *(v.x - baseline) + v.y*v.y + v.z*v.z);
    return v.z ? rtd : 0;
}

void z2rtd(const rs2::vertex * vertices, uint16_t * rtd,  rs2_intrinsics intrinsics, int baseline)
{
    for (auto i = 0;i < intrinsics.height*intrinsics.width; i++)
    {
        rtd[i] = get_pixel_rtd(vertices[i], baseline);
    }
}
template<typename T> 
T get_zo_point_value(const T * frame_data_in, rs2_intrinsics intrinsics, int zo_point_x, int zo_point_y , int patch_r)
{
    auto patch_size = (2 * patch_r + 1)*(2 * patch_r + 1);
    std::vector<T> values(patch_size);

    for (auto i = (zo_point_y - patch_r )* intrinsics.width; i < (zo_point_y + patch_r + 1)* intrinsics.width; i+= intrinsics.width)
    {
        for (auto j = (zo_point_x - patch_r); j < (zo_point_x + patch_r + 1); j++)
        {
            values.push_back(frame_data_in[i + j]);
        }
    }

    std::remove_if(values.begin(), values.end(), [](T val) {return val == 0;});
    std::sort(values.begin(), values.end());

    if (values.size() > 0)
        return values[values.size() / 2-1];
    return 0;
}

void detect_zero_order(const uint16_t * rtd, const uint16_t * depth_data_in, const uint8_t * ir_data, uint16_t * depth_data_out,
    rs2_intrinsics intrinsics, int rtd_high_threshold, int rtd_low_threshold, int ir_threshold,
    uint16_t zo_value, uint8_t iro_value)
{
    auto i_threshold_relative = (iro_value* ir_threshold) / 256;//(int)(((float)ir_threshold / 256.f) * iro_value);

    for (auto i = 0; i < intrinsics.height*intrinsics.width; i++)
    {
        auto rtd_val = rtd[i];
        auto ir_val = ir_data[i];

        if (depth_data_in[i] > 0 && ir_val < i_threshold_relative && rtd_val > (zo_value - rtd_low_threshold) && rtd_val < (zo_value + rtd_high_threshold))
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
    int zo_point_x = 314, int zo_point_y = 236, int ir_threshold = 115,
    int rtd_high_threshold = 200, int rtd_low_threshold = 200,
    int baseline = 31, int patch_r = 5)
{
    std::ifstream f;
    f.open("sample_1out_rtd.640x480.bin16", std::ios::binary);
    std::vector<uint16_t> rtd_ref(intrinsics.height*intrinsics.width*2);
    f.read((char*)rtd_ref.data(), intrinsics.height*intrinsics.width*2);
    std::vector<uint16_t> rtd( intrinsics.height*intrinsics.width);
    z2rtd(vertices, rtd.data(), intrinsics, baseline);
    auto zo_value = get_zo_point_value(rtd.data(), intrinsics, zo_point_x, zo_point_y, patch_r);
    auto iro_value = get_zo_point_value(ir_data, intrinsics, zo_point_x, zo_point_y, patch_r);
    detect_zero_order(rtd.data(), depth_data_in, ir_data, depth_data_out, intrinsics, rtd_high_threshold, rtd_low_threshold, ir_threshold, zo_value, iro_value);

}

void zero_order_fix_processor::process_frame(rs2::frameset data, rs2::frame_source & source)
{
    if (!_source_profile)
        _source_profile = data.get_depth_frame().get_profile();

    if (!_target_profile)
        _target_profile = _source_profile.clone(_source_profile.stream_type(), _source_profile.stream_index(), _source_profile.format());
    
    auto frames = get_frames(data);
    auto depth_frame = data.get_depth_frame();
    auto ir_frame = data.get_infrared_frame();

    auto points = _pc.calculate(depth_frame);
   
    auto out_frame = source.allocate_video_frame(_target_profile, depth_frame);
    auto depth_intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    zero_order_fix((const uint16_t*)depth_frame.get_data(), (const uint8_t*)ir_frame.get_data(), (uint16_t*)out_frame.get_data(), points.get_vertices(), depth_intrinsics);

    source.frame_ready(out_frame);
}

std::vector<rs2::frame> zero_order_fix_processor::get_frames(rs2::frameset frame)
{
    std::vector<rs2::frame> frames;
    if (auto fs = frame.as<rs2::frameset>())
    {
        for (auto f : fs)
            frames.push_back(f);
    }
    else
        frames.push_back(frame);
      
    return frames;
}

zero_order_fix_processor::zero_order_fix_processor()
    :rs2::processing_block([&](rs2::frame data, rs2::frame_source& source)
{
    process_frame(data, source);
})
{
    start(_queue);
}


