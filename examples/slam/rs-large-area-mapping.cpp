// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <cassert>
#include <iomanip>
#include <memory>
#include <fstream>
#include <unordered_map>
#include <string>
#include <cstring>
#include <unistd.h>
#include <future>
#include "rs-slam-trajectory.hpp"
#include "adjustment_layer.h"

struct to_string {
    std::ostringstream ss;
    template<class T> to_string & operator << (const T & val) { ss << val; return *this; }
    operator std::string() const { return ss.str(); }
};

static bool is_emitter_on(const rs2::frame &f) {
    auto stream = f.get_profile().stream_type();
    if (stream == RS2_STREAM_INFRARED || stream == RS2_STREAM_DEPTH) {
        if (f.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE))
            return f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE); //Laser power mode. Zero corresponds to Laser power switched off and one for switched on.
        else
            throw std::runtime_error(to_string() << "unabled to determine of emitter is enabled for infrared frame" << f.get_profile().stream_name());
    }
    return false;
}
void raw_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
{
    std::ofstream file(filename, std::ios::binary | std::ios::trunc);
    if (!file.good())
        throw std::runtime_error("Invalid binary file specified. Verify the target path and location permissions");
    file.write((char*)bytes.data(), bytes.size());
}

std::vector<uint8_t> bytes_from_raw_file(const std::string& filename)
{
    std::ifstream file(filename.c_str(), std::ios::binary);
    if (!file.good())
        throw std::runtime_error("Invalid binary file specified. Verify the source path and location permissions");

    // Determine the file length
    file.seekg(0, std::ios_base::end);
    std::size_t size = file.tellg();
    if (!size)
        throw std::runtime_error("Invalid binary file -zero-size");
    file.seekg(0, std::ios_base::beg);

    // Create a vector to store the data
    std::vector<uint8_t> v(size);

    // Load the data
    file.read((char*)&v[0], size);

    return v;
}

transformation to_transformation(const rs2_pose& p){
    return transformation{quaternion(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z), v3(p.translation.x,p.translation.y,p.translation.z)};
}

//static rs2::processing_block x;

int main(int c, char * v[]) try
{
    if (0) { usage: std::cerr << "Usage: " << v[0] << " [--serial <number>] [--enable-depth] [--use-depth] [--record-time <seconds>] [--stream] [--track] [--save-json <file.json>] [--play <file.bag>] [[--record] <file.rc>]\n"; return 1; }

        const char *recording_file = nullptr, *serial = nullptr, *calibration_json = nullptr, *playback_file = nullptr, *in_map_file = nullptr, *out_map_file = nullptr;
    bool stream = false, track = false, enable_depth = false, use_depth = true; double record_time_s = 0;

    for (int i=1; i<c; i++)
        if      (v[i][0] != '-' && !recording_file) recording_file = v[i];
        else if (strcmp(v[i], "--serial") == 0 && i+1 < c) serial = v[++i];
        else if (strcmp(v[i], "--save-json") == 0 && i+1 < c) calibration_json = v[++i];
        else if (strcmp(v[i], "--record") == 0 && i+1 < c) recording_file = v[++i];
        else if (strcmp(v[i], "--play") == 0 && i+1 < c) playback_file = v[++i];
        else if (strcmp(v[i], "--load-map") == 0 && i+1 < c) in_map_file = v[++i];
        else if (strcmp(v[i], "--save-map") == 0 && i+1 < c) out_map_file = v[++i];
        else if (strcmp(v[i], "--record-time") == 0 && i+1 < c) record_time_s = std::stod(v[++i]);
        else if (strcmp(v[i], "--track") == 0) track = true;
        else if (strcmp(v[i], "--stream") == 0) stream = true;
        else if (strcmp(v[i], "--enable-depth") == 0) enable_depth = true;
        else if (strcmp(v[i], "--use-depth") == 0) use_depth = true;
        else goto usage;

    if (!recording_file && !track && !stream)
        goto usage;

    rs2::context ctx;
    rs2::pipeline pipe(ctx);
    rs2::config cfg;

    for (const rs2::device &dev : ctx.query_devices())
        //for (const rs2::sensor &s : dev.query_sensors())
        std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << "\n";

    if (playback_file)
        cfg.enable_device_from_file(playback_file, false);
    else for (const rs2::device &d : ctx.query_devices()) {
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        break;
    }

    rs2::pipeline_profile pipeline_profile = cfg.resolve(pipe);
    auto pb = pipeline_profile.get_device().as<rs2::playback>();
    //if(pb){ pb.set_real_time(false); }

    // Initialize window for rendering
    window app(1280, 720, "RealSense Trajectory (Adjustment Layer) Example");
        
    // Construct an object to manage view state
    glfw_state app_state(0.0, 0.0);
    // Register callbacks to allow manipulation of the view state
    register_glfw_callbacks(app, app_state);
          
    // Get sensor
    std::shared_ptr<rs2::pose_sensor> tm_sensor;
    if(!playback_file)
        tm_sensor = std::make_shared<rs2::pose_sensor>(pipeline_profile.get_device().first<rs2::pose_sensor>());

    struct large_map_manager : public map_manager
    {
        large_map_manager(window& app_, rs2::pipeline& pipe_, std::shared_ptr<rs2::pose_sensor>& sensor_) : map_manager(app_, pipe_, sensor_) {}
        void calc_transform(const transformation& H, float mat[16]){ map_manager::calc_transform(rs2_quaternion{H.Q.x(),H.Q.y(),H.Q.z(),H.Q.w()}, rs2_vector{H.T.x(),H.T.y(),H.T.z()},mat);
        }
    } mmanager(app, pipe, tm_sensor);
    if(in_map_file){ mmanager.set_in_map_file_path(std::string(in_map_file)); mmanager.load_map();}
    if(out_map_file){ mmanager.set_out_map_file_path(std::string(out_map_file));}
        
    rs_adjustment_layer<large_map_manager> adjust(mmanager, tm_sensor);
        
    // Create objects for rendering the camera, the trajectory and the split screen
    camera_renderer cam_renderer;
    split_screen_renderer screen_renderer(app.width(), app.height(), mmanager, cam_renderer);

    pipe.start(cfg);
        
    // Main loop
    while (app)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        // process through the adjustment layer
        auto adjusted_data = adjust.process_tracker_pose(to_transformation(pose_data), f.get_timestamp(), nullptr);
        // buffer for GUI pose
        float r[16];
        
        if(adjusted_data.valid){
            // Calculate current adjusted transformation matrix
            mmanager.calc_transform(adjusted_data.H_adjusted_camera, r);
        }else{
            mmanager.tracker::calc_transform(pose_data, r);
        }
        // add to trajectory
        mmanager.add_to_trajectory(pose_data, r);
        // Draw the trajectory from different perspectives
        screen_renderer.draw_windows(app.width(), app.height(), app_state, r);
    }
        
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
