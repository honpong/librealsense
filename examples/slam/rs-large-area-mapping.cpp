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
        
    // Create objects for rendering the camera, the trajectory and the split screen
    camera_renderer cam_renderer;
        
    struct large_map_manager : public map_manager
    {
        large_map_manager(window& app_, rs2::pipeline& pipe_, std::shared_ptr<rs2::pose_sensor>& sensor_) : map_manager(app_, pipe_, sensor_)
        {}
        
        void set_in_map_file_path(const std::string& map_file) { in_map_file_path = map_file; }
        void set_out_map_file_path(const std::string& map_file) { out_map_file_path = map_file; }
        std::vector<std::pair<rs2_pose,tracked_point>>& get_nodes() { return nodes; }
        
        // Calculates transformation matrix based on pose data from the adjustment layer
        void calc_transform(const transformation& p, float mat[16])
        {
            rs2_quaternion q = {p.Q.x(), p.Q.y(), p.Q.z(), p.Q.w()};
            rs2_vector t = {p.T.x(), p.T.y(), p.T.z()};
            // Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns)
            mat[0] = -(1 - 2 * q.y*q.y - 2 * q.z*q.z); mat[4] = 2 * q.x*q.y - 2 * q.z*q.w;     mat[8] = -(2 * q.x*q.z + 2 * q.y*q.w);      mat[12] = t.x;
            mat[1] = -(2 * q.x*q.y + 2 * q.z*q.w);     mat[5] = 1 - 2 * q.x*q.x - 2 * q.z*q.z; mat[9] = -(2 * q.y*q.z - 2 * q.x*q.w);      mat[13] = t.y;
            mat[2] = -(2 * q.x*q.z - 2 * q.y*q.w);     mat[6] = 2 * q.y*q.z + 2 * q.x*q.w;     mat[10] = -(1 - 2 * q.x*q.x - 2 * q.y*q.y); mat[14] = t.z;
            mat[3] = 0.0f;                             mat[7] = 0.0f;                          mat[11] = 0.0f;                             mat[15] = 1.0f;
        }
        
        void add_to_trajectory(const rs2_pose& pose_data, const float r[16])
        {
            // Create a new point to be added to the trajectory
            tracked_point p{ rs2_vector{r[12], r[13], r[14]} , pose_data.tracker_confidence };
            // Register the new point
            tracker::add_to_trajectory(p);
            // Save latest pose data
            last_pose = {pose_data, p};
        }
        
    } mmanager(app, pipe, tm_sensor);
    
    if(in_map_file){ mmanager.set_in_map_file_path(std::string(in_map_file)); mmanager.load_map();}
    if(out_map_file){ mmanager.set_out_map_file_path(std::string(out_map_file));}
        
    split_screen_renderer screen_renderer(app.width(), app.height(), mmanager, cam_renderer);
    
    // Start pipeline with chosen configuration
    adjustment_layer adjust([&](adjustment_layer::landmark_t& landmark, void* handle) -> bool {
        rs2_vector pos;
        rs2_quaternion orient;
        landmark.valid = tm_sensor->get_static_node(landmark.guid, pos, orient);
        landmark.H_tracker_stage.Q = quaternion(orient.w, orient.x, orient.y, orient.z);
        landmark.H_tracker_stage.T = v3(pos.x,pos.y,pos.z);
        
        std::cout << "[T265   ] Get static node : " << landmark.guid << " at ";
        if(landmark.valid){ std::cout << landmark.H_tracker_stage.T << std::endl; }
        else { std::cout << " FAILED " << std::endl; }
        return landmark.valid;
    });
    adjust.set_switch_landmark_callback([&](adjustment_layer::landmark_t* landmark1, adjustment_layer::landmark_t* landmark0, adjustment_layer* adjust){
        auto str = [](adjustment_layer::landmark_t* l){ return l->guid+" ("+std::to_string(l->distance_to_camera)+"m)"; };
        std::cout << "[ADJUSTM] " << (landmark0? "Switch ref. from "+str(landmark0)+" to " : "Start ref. be ") << str(landmark1) << std::endl;
    });
    adjust.set_switch_landmark_threshold(0.2);
    
    // Add relocalization callback
    tm_sensor->set_notifications_callback([&](const rs2::notification& n) {
        if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
            std::cout << "Relocalization Event Detected." << std::endl;
            mmanager.load_nodes();
            adjust.reset_landmark();
            auto& nodes = mmanager.get_nodes();
            for(int i=0; i<(int)nodes.size(); ++i){
                auto stage = to_transformation(nodes[i].first);
                adjust.set_landmark(std::string("node")+std::to_string(i), stage.Q, stage.T);
                std::cout << "[ADJUSTM] Set landmark " << "node" << i << " at " << stage.T << std::endl;
            }
            adjust.config_landmark_switch_nearest(true);
            adjust.set_relocalization_linked_map_event();
        }
    });
    
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
            // add to trajectory
            mmanager.add_to_trajectory(pose_data, r);
            // Draw the trajectory from different perspectives
        }else{
            mmanager.tracker::calc_transform(pose_data, r);
            mmanager.add_to_trajectory(pose_data, r);
        }
        
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
