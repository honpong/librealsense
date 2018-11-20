// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <librealsense2/hpp/rs_internal.hpp> // Include RealSense Cross Platform API
#include <fstream>
#include "./../common/zero_order_fix.h"
#include "./../common/zero_order_fix.cpp"

//void copy(void* dst, void const* src, size_t size)
//{
//    auto from = reinterpret_cast<uint8_t const*>(src);
//    std::copy(from, from + size, reinterpret_cast<uint8_t*>(dst));
//}
//
//template<size_t SIZE>
//void rotate_270_degrees_clockwise(char * dest, const char * source, int width, int height)
//{
//    auto out = &dest[0];
//    for (int i = 0; i < height; ++i)
//    {
//        auto row = height-1 - i ;
//        for (int j = 0; j < width; ++j)
//        {
//            auto col = width-1 - j;
//            auto out_index = (row* width+ col )* SIZE;
//            copy((void*)(&out[out_index]),(void const* )(&source[(i* width + j) * SIZE]), SIZE);
//        }
//    }
//}
//
// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    
    //rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    // Create a simple OpenGL window for rendering:
    window app(1280, 1280, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, ir_image;
    rs2::software_device dev; // Create software-only device
    
    dev.create_matcher(RS2_MATCHER_DI);
    auto depth_sensor = dev.add_sensor("Depth"); // Define single sensor
    //rs2::recorder rec("C:/private/librealsense/build/examples/capture/1.bag", dev);
    auto depth_ref_sensor = dev.add_sensor("Depth_ref"); // Define single sensor
    
    rs2_intrinsics depth_intrinsics = { 640, 480, 312.1740, 247.3893, 558.9540, 572.0936, RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };
    auto depth_stream = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
                             640, 480, 60, 2,
                             RS2_FORMAT_Z16, depth_intrinsics });

    auto depth_stream_out = depth_ref_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 1,
                            640, 480, 60, 2,
                            RS2_FORMAT_Z16, depth_intrinsics });

    auto ir_stream = depth_sensor.add_video_stream({ RS2_STREAM_INFRARED, 1, 2,
                             640, 480, 60, 1,
                             RS2_FORMAT_Y8, depth_intrinsics });

    depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.000125);
    depth_sensor.open({ depth_stream,ir_stream });
    depth_ref_sensor.open(depth_stream_out);
    rs2::syncer s;
   
    depth_sensor.start(s);

    rs2::frame_queue q;
    depth_ref_sensor.start(q);
    std::ifstream ifile;
    ifile.open("sample_8in_z.640x480.bin16", std::ios::binary);

    std::vector<uint16_t> depth_frame(640 * 480 * 2);
    std::vector<char> depth_frame_ref(640 * 480 * 2);
    std::vector<char> ir_frame(640 * 480);

    /*std::vector<char> depth_frame_rotated(640 * 480 * 2);
    std::vector<char> depth_frame_ref_rotated(640 * 480 * 2);
    std::vector<char> ir_frame_rotated(640 * 480);*/

    int frame_number = 0;

    ifile.read((char*)depth_frame.data(), 640 * 480 * 2);
    ifile.close();

    ifile.open("sample_8out_z.640x480.bin16", std::ios::binary);
    ifile.read(depth_frame_ref.data(), 640 * 480 * 2);

    ifile.close();

    ifile.open("sample_8in_ir.640x480.bin8", std::ios::binary);
    ifile.read(ir_frame.data(), 640 * 480);

    /*rotate_270_degrees_clockwise<2>(depth_frame_rotated.data(), (char*)depth_frame.data(), 640, 480);
    rotate_270_degrees_clockwise<2>(depth_frame_ref_rotated.data(), depth_frame_ref.data(), 640, 480);
    rotate_270_degrees_clockwise<1>(ir_frame_rotated.data(), ir_frame.data(), 640, 480);*/

    rs2::colorizer c;
    rs2::zero_order_fix zo;
    while(app) // Application still alive?
    {
       
        depth_sensor.on_video_frame({ depth_frame.data(), // Frame pixels from capture API
            [](void*) {}, // Custom deleter (if required)
            640*2, 2, // Stride and Bytes-per-pixel
            (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
            depth_stream });

        
        depth_sensor.on_video_frame({ ir_frame.data(), // Frame pixels from capture API
         [](void*) {}, // Custom deleter (if required)
         640, 2, // Stride and Bytes-per-pixel
         (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
         ir_stream });

        depth_ref_sensor.on_video_frame({ depth_frame_ref.data(), // Frame pixels from capture API
            [](void*) {}, // Custom deleter (if required)
            640 * 2, 2, // Stride and Bytes-per-pixel
            (rs2_time_t)frame_number * 16, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, frame_number, // Timestamp, Frame# for potential sync services
            depth_stream_out });

        auto frames = s.wait_for_frames(); // Wait for next set of frames from the camera
        frame_number++;
        if (frames.size() < 2)
            continue;
        auto depth_out = q.wait_for_frame(); // Wait for next set of frames from the camera

       

        //auto ir = q.wait_for_frame(); // Wait for next set of frames from the camera
        auto depth_res = zo.process(frames);

        for (auto i = 0;i < 640 * 480; i++)
        {
            auto res = ((uint16_t*)depth_res.get_data())[i];
            auto out = ((uint16_t*)depth_out.get_data())[i];
            if (res != out)
            {
                std::cout << "fail";
            }

        }
        auto color_in = c.colorize(frames.get_depth_frame());
        auto color_out = c.colorize(depth_out);
        auto color_res = c.colorize(depth_res);

       /* auto color_in = frames.get_depth_frame();
        auto color_out = depth_out;
        auto color_res = depth_res;*/
        
        
        // Render depth on to the first half of the screen and color on to the second
        depth_image.render(color_in, { 0,               0, app.width() / 2, app.height()/2 });
        depth_image.render(color_out, { app.width() / 2, 0, app.width() / 2, app.height()/2 });
        ir_image.render(frames.get_infrared_frame(),  { 0,  app.height()/2 , app.width() / 2, app.height()/2});
        depth_image.render(color_res, { app.width() / 2,  app.height() / 2, app.width() / 2, app.height() / 2 });
        //color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });
    }
    //rec.resume();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
