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
#include "rc_tracker.h"

struct to_string {
    std::ostringstream ss;
    template<class T> to_string & operator << (const T & val) { ss << val; return *this; }
    operator std::string() const { return ss.str(); }
};

static rc_Extrinsics rc_from_rs(rs2_extrinsics rs) {
    rc_Extrinsics rc = {};
    for (int r = 0; r < 3; r++) {
        rc.pose_m.T.v[r] = rs.translation[r];
        rc.variance_m2.T.v[r] = 1e-6;
        rc.variance_m2.W.v[r] = 1e-6;
        for (int c = 0; c < 3; c++)
            rc.pose_m.R.v[c][r] = rs.rotation[r * 3 + c]; // column to row major
    }
    return rc;
}

template<typename In>
static typename std::enable_if<std::is_same<rc_GyroscopeIntrinsics,In>::value,rc_GyroscopeIntrinsics>::type rc_from_rs(rs2_motion_device_intrinsic rs) {
    rc_GyroscopeIntrinsics rc = {};
    for (int i=0; i < 3; i++) {
        for (int j=0; j<3; j++)
            rc.scale_and_alignment.v[i][j]    = rs.data[i][j];
        rc.bias_rad__s.v[i]                   = rs.data[i][3];
        rc.bias_variance_rad2__s2.v[i]        = rs.bias_variances[i];
        rc.measurement_variance_rad2__s2     += rs.noise_variances[i];
    }
    rc.measurement_variance_rad2__s2 /= 3;
    return rc;
}

template<typename In>
static typename std::enable_if<std::is_same<rc_AccelerometerIntrinsics,In>::value,rc_AccelerometerIntrinsics>::type rc_from_rs(rs2_motion_device_intrinsic rs) {
    rc_AccelerometerIntrinsics rc = {};
    for (int i=0; i < 3; i++) {
        for (int j=0; j<3; j++)
            rc.scale_and_alignment.v[i][j]   = rs.data[i][j];
        rc.bias_m__s2.v[i]                   = rs.data[i][3];
        rc.bias_variance_m2__s4.v[i]         = rs.bias_variances[i];
        rc.measurement_variance_m2__s4      += rs.noise_variances[i];
    }
    rc.measurement_variance_m2__s4 /= 3;
    return rc;
}

static rc_CameraIntrinsics rc_from_rs(rs2_intrinsics rs) {
    rc_CameraIntrinsics rc = {};
    rc.width_px = rs.width;
    rc.height_px = rs.height;
    rc.c_x_px = rs.ppx;
    rc.c_y_px = rs.ppy;
    rc.f_x_px = rs.fx;
    rc.f_y_px = rs.fy;
    switch (rs.model) {
        case RS2_DISTORTION_NONE: all_zero:  rc.type = rc_CALIBRATION_TYPE_UNDISTORTED;     for (int i = 0; i < 0; i++) rc.distortion[i] = rs.coeffs[i]; break;
        case RS2_DISTORTION_FTHETA:          rc.type = rc_CALIBRATION_TYPE_FISHEYE;         for (int i = 0; i < 1; i++) rc.distortion[i] = rs.coeffs[i]; break;
        case RS2_DISTORTION_KANNALA_BRANDT4: rc.type = rc_CALIBRATION_TYPE_KANNALA_BRANDT4; for (int i = 0; i < 4; i++) rc.distortion[i] = rs.coeffs[i]; break;
        case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
        case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
        case RS2_DISTORTION_BROWN_CONRADY:
            for (auto c : rs.coeffs) if (c) goto non_zero; goto all_zero;
        default: non_zero:
            throw std::runtime_error(to_string() << "unknown distortion model " << rs.model);
    }
    return rc;
}

static rc_Timestamp to_rc_Timestamp(const rs2::frame &f) {
    return rc_Timestamp(1000*f.get_timestamp());
}

static std::pair<rc_Timestamp, rc_Timestamp> to_rc_Timestamp_and_exposure(const rs2::frame& f) {
    auto p=std::make_pair(1000.0 * f.get_timestamp(), 0);
    if (f.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
        p.second = f.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
    else if (f.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP))
        p.first = f.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
    return p;
}

static bool is_emitter_on(const rs2::frame &f) {
    if (f.get_profile().stream_type() == RS2_STREAM_INFRARED) {
        if (f.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE))
            return f.get_frame_metadata(RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE); //Laser power mode. Zero corresponds to Laser power switched off and one for switched on.
        else
            throw std::runtime_error(to_string() << "unabled to determine of emitter is enabled for infrared frame" << f.get_profile().stream_name());
    }
    return false;
}

int main(int c, char * v[]) try
{
    if (0) { usage: std::cerr << "Usage: " << v[0] << " [--serial <number>] [--record-time <seconds>] [--track] [<file.rc>]\n"; return 1; }

    const char *recording_file = nullptr, *serial = nullptr, *calibration_json = nullptr, *playback_file = nullptr;
    bool track = false; double record_time_s = 0;

    for (int i=1; i<c; i++)
        if      (v[i][0] != '-' && !recording_file) recording_file = v[i];
        else if (strcmp(v[i], "--serial") == 0 && i+1 < c) serial = v[++i];
        else if (strcmp(v[i], "--save-json") == 0 && i+1 < c) calibration_json = v[++i];
        else if (strcmp(v[i], "--record") == 0 && i+1 < c) recording_file = v[++i];
        else if (strcmp(v[i], "--play") == 0 && i+1 < c) playback_file = v[++i];
        else if (strcmp(v[i], "--record-time") == 0 && i+1 < c) record_time_s = std::stod(v[++i]);
        else if (strcmp(v[i], "--track") == 0) track = true;
        else goto usage;

    if (!recording_file || !track)
        goto usage;

    rs2::context ctx;
    rs2::pipeline pipe(ctx);
    rs2::config cfg;

    for (const rs2::device &dev : ctx.query_devices())
        //for (const rs2::sensor &s : dev.query_sensors())
        std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << "\n";

    if (playback_file)
        cfg.enable_device_from_file(playback_file);
    else if (serial)
        cfg.enable_device(serial);
    //cfg.enable_all_streams();
    cfg.disable_stream(RS2_STREAM_COLOR);
    cfg.disable_stream(RS2_STREAM_DEPTH);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, RS2_FORMAT_Y8);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 2, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_POSE); // This is just configured for the sake of the pose reference

    std::unique_ptr<rc_Tracker, void (*)(rc_Tracker *)> rc = { rc_create(), rc_destroy };
    rs2::pipeline_profile pipeline_profile = cfg.resolve(pipe);
    std::unordered_map<int,int> sensor_id;

    rc_setMessageCallback(rc.get(), [](void *handle, rc_MessageLevel message_level, const char *message, size_t len) { std::cout.write(message,len); }, nullptr, rc_MESSAGE_INFO);

    rs2::stream_profile ref; try { ref = pipeline_profile.get_stream(RS2_STREAM_POSE); } catch (...) { ref = pipeline_profile.get_stream(RS2_STREAM_ACCEL); };

    {
        int cameras = 0, accels = 0, gyros = 0, depths = 0;
        rs2_stream prev_image_type; int prev_image_id = -2;
        for (rs2::stream_profile &s : pipeline_profile.get_streams()) {
            auto ex = rc_from_rs(s.get_extrinsics_to(ref));
            std::cout << s.stream_name() << "\n";
            switch (s.stream_type()) {
            case RS2_STREAM_POSE: break; // added only for the extrinsics reference
            case RS2_STREAM_FISHEYE:
            case RS2_STREAM_INFRARED: {
                rc_CameraIntrinsics in = rc_from_rs(s.as<rs2::video_stream_profile>().get_intrinsics());
                if (s.format() != RS2_FORMAT_Y8)
                    throw std::runtime_error(to_string() << "unsupported image format " << s);
                if (!rc_configureCamera(rc.get(), cameras, rc_FORMAT_GRAY8, &ex, &in))
                    throw std::runtime_error(to_string() << "unabled to configure camera " << s);
                auto v = s.as<rs2::video_stream_profile>();
                if (prev_image_id/2 == cameras/2 && prev_image_type == v.stream_type())
                    if (!rc_configureStereo(rc.get(), prev_image_id, cameras))
                        throw std::runtime_error(to_string() << "error configuring stereo streams" << s);
                prev_image_type = v.stream_type();
                prev_image_id = cameras;
                sensor_id[s.unique_id()] = cameras++;
            }   break;
            case RS2_STREAM_DEPTH: {
                rc_CameraIntrinsics in = rc_from_rs(s.as<rs2::video_stream_profile>().get_intrinsics());
                if (s.format() != RS2_FORMAT_Z16)
                    throw std::runtime_error(to_string() << "unsupported image format " << s);
                if (!rc_configureCamera(rc.get(), depths, rc_FORMAT_DEPTH16, &ex, &in))
                    throw std::runtime_error(to_string() << "unabled to configure camera " << s);
                sensor_id[s.unique_id()] = depths++;
            }   break;
            case RS2_STREAM_ACCEL: {
                rc_AccelerometerIntrinsics in = { {{{1,0,0},{0,1,0},{0,0,1}}}, {{0,0,0}} };
                try {
                    in = rc_from_rs<rc_AccelerometerIntrinsics>(s.as<rs2::motion_stream_profile>().get_motion_intrinsics());
                } catch(...) {
                    std::cerr << "Acceleromter intrinsics unavailble; using defaults. Consider using rs-imu-calibration.\n";
                }
                for (auto &bv : in.bias_variance_m2__s4.v)
                    if (!bv) bv = 1e-3f;
                if (!in.measurement_variance_m2__s4)
                    in.measurement_variance_m2__s4 = 0.00006695245247101411;
                if (!rc_configureAccelerometer(rc.get(), accels, &ex, &in))
                    throw std::runtime_error(to_string() << "unabled to configure accelerometer " << s);
                sensor_id[s.unique_id()] = accels++;
            }   break;
            case RS2_STREAM_GYRO: {
                rc_GyroscopeIntrinsics in = { {{{1,0,0},{0,1,0},{0,0,1}}}, {{0,0,0}} };
                try {
                    in = rc_from_rs<rc_GyroscopeIntrinsics>(s.as<rs2::motion_stream_profile>().get_motion_intrinsics());
                } catch(...) {
                    std::cerr << "Gyroscope intrinsics unavailble; using defaults. Consider using rs-imu-calibration.\n";
                }
                for (auto &bv : in.bias_variance_rad2__s2.v)
                    if (!bv) bv = 1e-4f;
                if (!in.measurement_variance_rad2__s2)
                    in.measurement_variance_rad2__s2 = 0.000005148030140844639;
                if (!rc_configureGyroscope(rc.get(), gyros, &ex, &in))
                    throw std::runtime_error(to_string() << "unabled to configure gyroscope " << s);
                sensor_id[s.unique_id()] = gyros++;
            }   break;
            default:
                throw std::runtime_error(to_string() << "unknown stream type configred for slam: " << s.stream_name());
            }
        }
        if (accels < 1 || gyros < 1 || cameras < 1)
            throw std::runtime_error(to_string() << "slam requires at least one grey scale camera, one gyroscope and one accelerometer."
                                      " found "<< cameras << " camera(s), " << gyros << " gyroscope(s), " << accels << " accelerometer(s).");
    }

    if (calibration_json) {
        const char *json = nullptr;
        if (size_t json_size = rc_getCalibration(rc.get(), &json))
            std::ofstream(calibration_json) << json << "\n";
    }

    rc_configureQueueStrategy(rc.get(), rc_QUEUE_MINIMIZE_LATENCY);

    rs2::software_device dev;
    struct rc_software_pose {
        rs2::software_sensor pose_sensor;
        rs2::stream_profile pose_stream;
        rc_DataPath path;
        rc_SensorType output_type;
        int frame_number = 0;
        rc_software_pose(const char *name, rs2::software_device &dev, const rs2::stream_profile &ref, int output_fps, rc_SensorType output_type_, rc_Tracker *tracker, rc_DataPath path_)
            : pose_sensor(dev.add_sensor(name)),
              pose_stream(pose_sensor.add_pose_stream({ RS2_STREAM_POSE, 0 /*index???*/, 0 /*unique_id???*/, output_fps, RS2_FORMAT_6DOF })),
              path(path_), output_type(output_type_) {
          pose_stream.register_extrinsics_to(ref, {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
          //pose_sensor.open(pose_stream);
        }
        static void data_callback(void *handle, rc_Tracker *tracker, const rc_Data *data) {
          auto &sp = *(rc_software_pose *)handle;
          if (data->path == sp.path && data->type == sp.output_type) {
              rc_PoseVelocity v;
              rc_PoseAcceleration a;
              rc_PoseTime pose_time = rc_getPose(tracker, &v, &a, data->path);

              rs2_software_pose_frame::pose_frame_info pose = {};
              for (int i=0; i<4; i++)
                pose.rotation[i] = pose_time.pose_m.Q.v[i];
              for (int i=0; i<3; i++) {
                pose.translation[i] = pose_time.pose_m.T.v[i];
                pose.velocity[i] = v.T.v[i];
                pose.acceleration[i] = a.T.v[i];
                pose.angular_velocity[i] = v.W.v[i];
                pose.angular_acceleration[i] = a.W.v[i];
              }
              pose.tracker_confidence = rc_getConfidence(tracker);
              //pose.mapper_confidence = ;

              rs2_software_pose_frame frame = {};
              frame.data = (void *)new rs2_software_pose_frame::pose_frame_info(pose);
              frame.deleter = [](void *pf) { delete (rs2_software_pose_frame::pose_frame_info *)pf; };
              frame.timestamp = (double)pose_time.time_us / 1000;
              frame.domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME; // FIXME: this should come from the input sensors
              frame.frame_number = sp.frame_number++;
              frame.profile = sp.pose_stream.get();

              sp.pose_sensor.on_pose_frame(frame);
          }
        }
    };

    int output_fps = pipeline_profile.get_stream(RS2_STREAM_GYRO).fps(); // we choose to output pose on the gyro, see rc_setDataCallback
    struct fast_slow { rc_software_pose fast, slow; } fast_slow = {
        {"Fast Pose", dev, ref, output_fps,     rc_SENSOR_TYPE_GYROSCOPE, rc.get(), rc_DATA_PATH_FAST},
        {"Slow Pose", dev, ref, output_fps / 2, rc_SENSOR_TYPE_GYROSCOPE, rc.get(), rc_DATA_PATH_SLOW},
    };
    rc_setDataCallback(rc.get(), [](void *handle, rc_Tracker *tracker, const rc_Data *data) {
        auto &fs = *(struct fast_slow*)handle;
        if      (data->path == rc_DATA_PATH_FAST) rc_software_pose::data_callback((void*)&fs.fast, tracker, data);
        else if (data->path == rc_DATA_PATH_SLOW) rc_software_pose::data_callback((void*)&fs.slow, tracker, data);
    }, (void *)&fast_slow);

    // either or both of these
    if (recording_file) rc_startCapture(rc.get(), rc_RUN_ASYNCHRONOUS, [](void *handle, const void *buffer, size_t length) {
        if (buffer)
            static_cast<std::ofstream*>(handle)->write((const char *)buffer, length);
        else
            delete (std::ofstream*)handle;
    }, (void*)new std::ofstream(recording_file, std::ios::binary));
    //rc_setOutputLog(rc.get(), "/tmp/t.rc", rc_RUN_ASYNCHRONOUS);
    if (track) rc_startTracker(rc.get(), rc_RUN_ASYNCHRONOUS | rc_RUN_FAST_PATH | rc_RUN_RELOCALIZATION | rc_RUN_POSE_JUMP);

    pipe.start(cfg, [&rc,&sensor_id](const rs2::frame& frame) {
        try {
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
                    assert(id0/2 == id1/2 && f0.get_timestamp() == f1.get_timestamp() && f0.get_width() == f1.get_width() && f0.get_height() == f1.get_height());
                    if (is_emitter_on(f0) || is_emitter_on(f1)) std::cout << "skipping stereo pair with emitter on\n"; else
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
                    if (is_emitter_on(v)) std::cout << "skipping " << v.get_profile().stream_name() << " with emitter on\n"; else
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
                if (is_emitter_on(v)) std::cout << "skipping " << v.get_profile().stream_name() << " with emitter on\n"; else
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
            }
        }
        } catch (const rs2::error & e) {
            std::cerr << "callback failed " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "calback failed " << e.what() << std::endl;
        }
    });

    if (record_time_s)
        std::this_thread::sleep_for(std::chrono::milliseconds(uint64_t(1000*record_time_s)));
    else if (isatty(STDIN_FILENO))
        { std::cerr << "press return to finish recording:"; std::getchar(); }

    pipe.stop();
    rc_stopTracker(rc.get());

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
