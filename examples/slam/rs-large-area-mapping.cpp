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

//static rs2::processing_block x;

int main(int c, char * v[]) try
{
    if (0) { usage: std::cerr << "Usage: " << v[0] << " [--serial <number>] [--enable-depth] [--use-depth] [--record-time <seconds>] [--stream] [--track] [--save-json <file.json>] [--play <file.bag>] [[--record] <file.rc>]\n"; return 1; }

    const char *recording_file = nullptr, *serial = nullptr, *calibration_json = nullptr, *playback_file = nullptr;
    bool stream = false, track = false, enable_depth = false, use_depth = true; double record_time_s = 0;

    for (int i=1; i<c; i++)
        if      (v[i][0] != '-' && !recording_file) recording_file = v[i];
        else if (strcmp(v[i], "--serial") == 0 && i+1 < c) serial = v[++i];
        else if (strcmp(v[i], "--save-json") == 0 && i+1 < c) calibration_json = v[++i];
        else if (strcmp(v[i], "--record") == 0 && i+1 < c) recording_file = v[++i];
        else if (strcmp(v[i], "--play") == 0 && i+1 < c) playback_file = v[++i];
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
        /**
        const char *d_serial = d.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : nullptr;
        if (serial && (!d_serial || 0 != strcmp(serial, d_serial)))
            continue;
        if (d_serial)
            cfg.enable_device(d_serial);
        for (const rs2::sensor &s : d.query_sensors()) {
            for (const rs2::stream_profile &p : s.get_stream_profiles()) {
                if      (p.stream_type() == RS2_STREAM_GYRO)                  cfg.enable_stream(RS2_STREAM_GYRO,     p.stream_index());
                else if (p.stream_type() == RS2_STREAM_ACCEL)                 cfg.enable_stream(RS2_STREAM_ACCEL,    p.stream_index());
                else if (p.stream_type() == RS2_STREAM_DEPTH && enable_depth) cfg.enable_stream(RS2_STREAM_DEPTH,    p.stream_index());
                else if (p.stream_type() == RS2_STREAM_INFRARED)              cfg.enable_stream(RS2_STREAM_INFRARED, p.stream_index(), RS2_FORMAT_Y8);
                else if (p.stream_type() == RS2_STREAM_FISHEYE)               cfg.enable_stream(RS2_STREAM_FISHEYE,  p.stream_index(), RS2_FORMAT_Y8);
                else if (p.stream_type() == RS2_STREAM_POSE)                  cfg.enable_stream(RS2_STREAM_POSE,     p.stream_index()); // This is just configured for the sake of the pose reference
            }
        }
         */
        //cfg.enable_stream(RS2_STREAM_GYRO);
        //cfg.enable_stream(RS2_STREAM_ACCEL);
        //cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
        //cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
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
    map_manager mmanager(app, pipe, tm_sensor);
    split_screen_renderer screen_renderer(app.width(), app.height(), mmanager, cam_renderer);
    
    struct ui_handles {
        window* app;
        glfw_state* app_state;
        map_manager* mmanager;
        split_screen_renderer* screen_renderer;
        float r[16];
    } hlds{ &app, &app_state, &mmanager, &screen_renderer };
        
#if 0
     rc_setDataCallback(rc.get(), [](void *handle, rc_Tracker *tracker, const rc_Data * data){
         if (data->type == rc_SENSOR_TYPE_GYROSCOPE && data->path == rc_DATA_PATH_FAST) {
             rc_PoseVelocity v;
             rc_PoseAcceleration a;
             rc_PoseTime pose_time = rc_getPose(tracker, &v, &a, data->path);

             rs2_pose pose_data;
             pose_data.rotation.w = pose_time.pose_m.Q.w;
             pose_data.rotation.x = pose_time.pose_m.Q.x;
             pose_data.rotation.y = pose_time.pose_m.Q.y;
             pose_data.rotation.z = pose_time.pose_m.Q.z;
             pose_data.translation.x = pose_time.pose_m.T.x;
             pose_data.translation.y = pose_time.pose_m.T.y;
             pose_data.translation.z = pose_time.pose_m.T.z;
             pose_data.velocity.x = v.T.x;
             pose_data.velocity.y = v.T.y;
             pose_data.velocity.z = v.T.z;
             pose_data.acceleration.x = a.T.x;
             pose_data.acceleration.y = a.T.y;
             pose_data.acceleration.z = a.T.z;
             pose_data.angular_velocity.x = v.W.x;
             pose_data.angular_velocity.y = v.W.y;
             pose_data.angular_velocity.z = v.W.z;
             pose_data.angular_acceleration.x = a.W.x;
             pose_data.angular_acceleration.y = a.W.y;
             pose_data.angular_acceleration.z = a.W.z;
             pose_data.tracker_confidence = rc_getConfidence(tracker);
             
             float r[16];
            auto hlds = (ui_handles*)handle;
             // Calculate current transformation matrix
            hlds->mmanager->calc_transform(pose_data, r);
            // add to trajectory
            hlds->mmanager->add_to_trajectory(pose_data, r);
            // Draw the trajectory from different perspectives
            //hlds->screen_renderer->draw_windows(hlds->app->width(), hlds->app->height(), *hlds->app_state, r);
         }
     }, (void *)&hlds);
#endif
    
     // Start pipeline with chosen configuration
#if 1
    
    adjustment_layer adjust([&](adjustment_layer::landmark_t& landmark, void* handle) -> bool
    {
        rs2_vector pos;
        rs2_quaternion orient;
        bool sts = tm_sensor->get_static_node(landmark.guid, pos, orient);
        landmark.H_tracker_stage.Q = quaternion(orient.w, orient.x, orient.y, orient.z);
        landmark.H_tracker_stage.T = v3(pos.x,pos.y,pos.z);
        
        return sts;
    });
    adjust.set_switch_landmark_callback([&](adjustment_layer::landmark_t* landmark1, adjustment_layer::landmark_t* landmark0, adjustment_layer* adjust){
        auto str = [](adjustment_layer::landmark_t* l){ return l->guid+" ("+std::to_string(l->distance_to_camera)+"m)"; };
        std::cout << "[ADJUSTM] " << (landmark0? "Switch ref. from "+str(landmark0)+" to " : "Start ref. be ") << str(landmark1) << std::endl;
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

        float r[16];
        // Calculate current transformation matrix
        mmanager.calc_transform(pose_data, r);
        // add to trajectory
        mmanager.add_to_trajectory(pose_data, r);
        // Draw the trajectory from different perspectives
        screen_renderer.draw_windows(app.width(), app.height(), app_state, r);
    }
#else
    pipe.start(cfg, [&rc,&sensor_id, &use_depth](const rs2::frame& frame) {
        try {
            /*
            if(frame.is<rs2::frameset>()){
                frame.as<rs2::frameset>().foreach_rs([](const rs2::frame& f){
                    printf("frame (set) number %llu %f %s %d\n", f.get_frame_number(), f.get_timestamp(), rs2_format_to_string(f.get_profile().format()), f.get_profile().stream_index());
                });
            }else{ printf("frame (   ) number %llu %f %s\n", frame.get_frame_number(), frame.get_timestamp(), rs2_format_to_string(frame.get_profile().format())); }
            */
        if (frame.is<rs2::frameset>()) {
            auto fs = frame.as<rs2::frameset>();
            for (auto fi = fs.begin(), ni=fi; fi != fs.end(); ++fi) {
                rs2::frame f = (*fi);
                
                rs2::frame n; if (++(ni=fi) != fs.end()) n = *ni;
                rs2::stream_profile fp = f.get_profile();
                rs2::stream_profile np; if (n) np = n.get_profile();
                if (fp.format() == RS2_FORMAT_Y8 && np && np.format() == RS2_FORMAT_Y8) {
                    auto f0 = f.as<rs2::video_frame>(); auto p0 = fp.as<rs2::video_stream_profile>(); auto id0 = sensor_id[p0.unique_id()];
                    auto f1 = n.as<rs2::video_frame>(); auto p1 = np.as<rs2::video_stream_profile>(); auto id1 = sensor_id[p1.unique_id()];
                    if(f0.get_timestamp()!=f1.get_timestamp()){
                        printf("timestamp mismatch %f %f\n", f0.get_timestamp(), f1.get_timestamp());
                        //printf("frameset size %d\n",fs.size());
                        continue;
                    }
                    assert(id0/2 == id1/2 && f0.get_timestamp() == f1.get_timestamp() && f0.get_width() == f1.get_width() && f0.get_height() == f1.get_height());
                    if (is_emitter_on(f0) || is_emitter_on(f1)) std::cout << "    skipping stereo pair with emitter on\n"; else
                    if (!rc_receiveStereo(rc.get(), id0/2, rc_FORMAT_GRAY8,
                                          to_rc_Timestamp_and_exposure(fs).first, to_rc_Timestamp_and_exposure(fs).second,
                                          f0.get_width(), f0.get_height(), f0.get_stride_in_bytes(), f1.get_stride_in_bytes(),
                                          f0.get_data(), f1.get_data(),
                                          [](void *f){ delete (rs2::frameset*)f; }, (void*)new rs2::frameset(fs)))
                        throw std::runtime_error(to_string() << "failed stereo receive for frame " << fs.get_profile());
                    fi = ni;
                } else if (fp.format() == RS2_FORMAT_Y8 || fp.format() == RS2_FORMAT_Z16) {
                    const auto &v = f.as<rs2::video_frame>();
                    const auto &p = fp.as<rs2::video_stream_profile>();
                    bool on = is_emitter_on(v);
                    if ((fp.format() == RS2_FORMAT_Y8 && on) || (fp.format() == RS2_FORMAT_Z16 && (!on || !use_depth))) std::cout << "    skipping " << v.get_profile().stream_name() << " with emitter " << (on ? "on" : "off") << "\n"; else
                    if (!rc_receiveImage(rc.get(), sensor_id[p.unique_id()], p.format() == RS2_FORMAT_Y8 ? rc_FORMAT_GRAY8 : rc_FORMAT_DEPTH16,
                                         to_rc_Timestamp_and_exposure(v).first, to_rc_Timestamp_and_exposure(v).second,
                                         v.get_width(), v.get_height(), v.get_stride_in_bytes(),
                                         v.get_data(),
                                         [](void *f){ delete (rs2::frame*)f; }, (void*)new rs2::frame(frame)))
                        throw std::runtime_error(to_string() << "failed receive for frame " << fp);
                } else if (fp.stream_type() == RS2_STREAM_COLOR)
                    ; // skipping for now
                else
                    throw std::runtime_error(to_string() << "unexpected frameset " << fp);
            }
        } else {
            auto s = frame.get_profile();
            if (s.format() == RS2_FORMAT_Y8 || s.format() == RS2_FORMAT_Z16) {
                const rs2::video_frame &v = frame.as<rs2::video_frame>();
                const rs2::video_stream_profile &p = s.as<rs2::video_stream_profile>();
                bool on = is_emitter_on(v);
                if ((s.format() == RS2_FORMAT_Y8 && on) || (s.format() == RS2_FORMAT_Z16 && (!on || !use_depth))) std::cout << "    skipping " << v.get_profile().stream_name() << " with emitter " << (on ? "on" : "off") << "\n"; else
                if (!rc_receiveImage(rc.get(), sensor_id[p.unique_id()], p.format() == RS2_FORMAT_Y8 ? rc_FORMAT_GRAY8 : rc_FORMAT_DEPTH16,
                                     to_rc_Timestamp_and_exposure(v).first, to_rc_Timestamp_and_exposure(v).second,
                                     v.get_width(), v.get_height(), v.get_stride_in_bytes(),
                                     v.get_data(),
                                     [](void *f){ delete (rs2::frame*)f; }, (void*)new rs2::frame(frame)))
                    throw std::runtime_error(to_string() << "failed receive for frame " << s);
            } else if (s.format() == RS2_FORMAT_MOTION_XYZ32F) {
                const auto &m = frame.as<rs2::motion_frame>();
                const auto &p = s.as<rs2::motion_stream_profile>();
                const auto d = m.get_motion_data();
                if (s.stream_type() == RS2_STREAM_GYRO) {
                    if (!rc_receiveGyro(rc.get(), sensor_id[p.unique_id()], to_rc_Timestamp(m), rc_Vector{{ d.x, d.y, d.z }}))
                        throw std::runtime_error(to_string() << "failed receive for frame " << s);
                } else if (s.stream_type() == RS2_STREAM_ACCEL) {
                    if (!rc_receiveAccelerometer(rc.get(), sensor_id[p.unique_id()], to_rc_Timestamp(m), rc_Vector{{ d.x, d.y, d.z }}))
                        throw std::runtime_error(to_string() << "failed receive for frame " << s);
                }
            } else if (s.format() == RS2_FORMAT_6DOF) {
            } else
                throw std::runtime_error(to_string() << "unexpected frame " << s);
        }
        } catch (const rs2::error & e) {
            std::cerr << "callback failed " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "calback failed " << e.what() << std::endl;
        }
    });

    if (record_time_s)
        std::this_thread::sleep_for(std::chrono::milliseconds(uint64_t(1000*record_time_s)));
    else if (isatty(STDIN_FILENO)){
        
        bool firstframe = false;
        auto is_playback_or_live = [&](){
            if(!pb){ return true; }
            if(!firstframe && pb.current_status() == RS2_PLAYBACK_STATUS_PLAYING){ return (firstframe = true);}
            return (!firstframe || pb.current_status() == RS2_PLAYBACK_STATUS_PLAYING);
        };
        while(app && is_playback_or_live()){
            //hlds->screen_renderer->draw_windows(hlds->app->width(), hlds->app->height(), *hlds->app_state, r);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            // Draw the trajectory from different perspectives
            float r[16];
            screen_renderer.draw_windows(app.width(), app.height(), app_state, mmanager.get_last_r(r));
        }
    }
        
    pipe.stop();
        
#endif

#if 0
    struct save_map_t
    {
        save_map_t(rc_Tracker* tracker, std::string filename) {
            outfile.open(filename, std::ios_base::binary);
            if(!outfile.fail()){
                std::cout << " writing map file " << filename << std::endl;
                rc_saveMap(tracker, [](void *handle, const void* buffer, size_t length){
                    auto* save_map = (save_map_t*)handle;
                    if(length){
                        std::cout << "writing " << length << " bytes" << std::endl;
                        save_map->outfile.write((const char*)buffer, length);
                    }
                    else {
                        save_map->outfile.close();
                        save_map->task.set_value();
                    };
                }, this);
            }
            task_done.wait();
            std::cout << "end writing map file " << std::endl;
        }
        std::promise<void> task;
        std::future<void> task_done = task.get_future();
        std::ofstream outfile;
    } save_map(rc.get(), std::string(playback_file?playback_file:"temp") + ".map" );

    if (recording_file)
        printf("pipe stoppped; flushing recording\n");
    rc_stopTracker(rc.get());
#endif
        
    while(pb && app){ std::this_thread::sleep_for(std::chrono::milliseconds(33)); }
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
