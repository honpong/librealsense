// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <thread>
#include <iomanip>

int main(int argc, char * argv[]) try
{
    rs2::context ctx;
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    pipe.start(cfg, [](const rs2::frame& frame) {
        std::cout << (long)frame.get_timestamp() << " " << frame.get_profile().stream_name() << "\n";
    });

    for(;;) std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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
