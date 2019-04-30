// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <cassert>
#include <iomanip>
#include <memory>
#include <fstream>
#include <unordered_map>
#include "rc_tracker.h"

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
            throw std::runtime_error((std::stringstream() << "unknown distortion model " << rs.model).str());
    }
    return rc;
}

int main(int argc, char * argv[]) try
{
    rs2::context ctx;
    rs2::pipeline pipe(ctx);
    rs2::config cfg;

    for (const rs2::device &dev : ctx.query_devices())
        //for (const rs2::sensor &s : dev.query_sensors())
        std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << "\n";


    //cfg.enable_all_streams();
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_POSE); // This is just configured for the sake of the pose reference

    std::unique_ptr<rc_Tracker, void (*)(rc_Tracker *)> rc = { rc_create(), rc_destroy };
    rs2::pipeline_profile pipeline_profile = cfg.resolve(pipe);
    std::unordered_map<int,int> sensor_id;
    auto to_rc_Timestamp = [](const rs2::frame &f) -> rc_Timestamp {
        return rc_Timestamp(1000*f.get_timestamp());
    };

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
                    throw std::runtime_error((std::stringstream() << "unsupported image format " << s) .str());
                if (!rc_configureCamera(rc.get(), cameras, rc_FORMAT_GRAY8, &ex, &in))
                    throw std::runtime_error((std::stringstream() << "unabled to configure camera " << s) .str());
                auto v = s.as<rs2::video_stream_profile>();
                if (prev_image_id/2 == cameras/2 && prev_image_type == v.stream_type())
                    if (!rc_configureStereo(rc.get(), prev_image_id, cameras))
                        throw std::runtime_error((std::stringstream() << "error configuring stereo streams" << s).str());
                prev_image_type = v.stream_type();
                prev_image_id = cameras;
                sensor_id[s.unique_id()] = cameras++;
            }   break;
            case RS2_STREAM_DEPTH: {
                rc_CameraIntrinsics in = rc_from_rs(s.as<rs2::video_stream_profile>().get_intrinsics());
                if (s.format() != RS2_FORMAT_Z16)
                    throw std::runtime_error((std::stringstream() << "unsupported image format " << s) .str());
                if (!rc_configureCamera(rc.get(), depths, rc_FORMAT_DEPTH16, &ex, &in))
                    throw std::runtime_error((std::stringstream() << "unabled to configure camera " << s) .str());
                sensor_id[s.unique_id()] = depths++;
            }   break;
            case RS2_STREAM_ACCEL: {
                auto in = rc_from_rs<rc_AccelerometerIntrinsics>(s.as<rs2::motion_stream_profile>().get_motion_intrinsics());
                if (!rc_configureAccelerometer(rc.get(), accels, &ex, &in))
                    throw std::runtime_error((std::stringstream() << "unabled to configure accelerometer " << s).str());
                sensor_id[s.unique_id()] = accels++;
            }   break;
            case RS2_STREAM_GYRO: {
                auto in = rc_from_rs<rc_GyroscopeIntrinsics>(s.as<rs2::motion_stream_profile>().get_motion_intrinsics());
                if (!rc_configureGyroscope(rc.get(), gyros, &ex, &in))
                    throw std::runtime_error((std::stringstream() << "unabled to configure gyroscope " << s).str());
                sensor_id[s.unique_id()] = gyros++;
            }   break;
            default:
                throw std::runtime_error((std::stringstream() << "unknown stream type configred for slam: " << s.stream_name()).str());
            }
        }
        if (accels < 1 || gyros < 1 || cameras < 1)
            throw std::runtime_error((std::stringstream() << "slam requires at least one grey scale camera, one gyroscope and one accelerometer."
                                      " found "<< cameras << " camera(s), " << gyros << " gyroscope(s), " << accels << " accelerometer(s).").str());
    }

    {
        const char *json = nullptr;
        if (size_t json_size = rc_getCalibration(rc.get(), &json))
            std::cout << json << "\n";
    }

    rc_configureQueueStrategy(rc.get(), rc_QUEUE_MINIMIZE_LATENCY);

    // either or both of these
    rc_startCapture(rc.get(), rc_RUN_ASYNCHRONOUS, [](void *handle, const void *buffer, size_t length) {
        if (buffer)
            static_cast<std::ofstream*>(handle)->write((const char *)buffer, length);
        else
            delete (std::ofstream*)handle;
    }, (void*)new std::ofstream("/tmp/t.rc", std::ios::binary));
    //rc_setOutputLog(rc.get(), "/tmp/t.rc", rc_RUN_ASYNCHRONOUS);
    //rc_startTracker(rc.get(), rc_RUN_ASYNCHRONOUS);

    pipe.start(cfg, [&rc,&sensor_id,&to_rc_Timestamp](const rs2::frame& frame) {
        try {
        if (frame.is<rs2::frameset>()) {
            auto fs = frame.as<rs2::frameset>();
            if (fs.size() == 2 && fs[0].get_profile().format() == RS2_FORMAT_Y8 && fs[1].get_profile().format() == RS2_FORMAT_Y8) {
                auto f0 = fs[0].as<rs2::video_frame>(); auto p0 = f0.get_profile().as<rs2::video_stream_profile>(); auto id0 = sensor_id[p0.unique_id()];
                auto f1 = fs[1].as<rs2::video_frame>(); auto p1 = f0.get_profile().as<rs2::video_stream_profile>(); auto id1 = sensor_id[p1.unique_id()];
                assert(id0/2 == id1/2);
                if (!rc_receiveStereo(rc.get(), id0/2, rc_FORMAT_GRAY8,
                                      to_rc_Timestamp(fs), (rc_Timestamp)fs.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE),
                                      f0.get_width(), f0.get_height(), f0.get_stride_in_bytes(), f1.get_stride_in_bytes(),
                                      f0.get_data(), f1.get_data(),
                                      [](void *f){ delete (rs2::frameset*)f; }, (void*)new rs2::frameset(fs)))
                    throw std::runtime_error((std::stringstream() << "failed stereo receive for frame " << fs).str());
            } else {
                for (const rs2::frame &frame : frame.as<rs2::frameset>()) {
                    auto s = frame.get_profile();
                    if (s.format() == RS2_FORMAT_Y8 || s.format() == RS2_FORMAT_Z16) {
                        const auto &v = frame.as<rs2::video_frame>();
                        const auto &p = s.as<rs2::video_stream_profile>();
                        if (!rc_receiveImage(rc.get(), sensor_id[p.unique_id()], p.format() == RS2_FORMAT_Y8 ? rc_FORMAT_GRAY8 : rc_FORMAT_DEPTH16,
                                             to_rc_Timestamp(v), (rc_Timestamp)v.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE),
                                             v.get_height(), v.get_width(), v.get_stride_in_bytes(),
                                             v.get_data(),
                                             [](void *f){ delete (rs2::frame*)f; }, (void*)new rs2::frame(frame)))
                            throw std::runtime_error((std::stringstream() << "failed receive for frame " << frame).str());
                    } else
                        throw  std::runtime_error((std::stringstream() << "unexpected frameset " << frame).str());
                }
            }
        } else {
            auto s = frame.get_profile();
            if (s.format() == RS2_FORMAT_Y8 || s.format() == RS2_FORMAT_Z16) {
                const rs2::video_frame &v = frame.as<rs2::video_frame>();
                const rs2::video_stream_profile &p = s.as<rs2::video_stream_profile>();
                if (!rc_receiveImage(rc.get(), sensor_id[p.unique_id()], p.format() == RS2_FORMAT_Y8 ? rc_FORMAT_GRAY8 : rc_FORMAT_DEPTH16,
                                     to_rc_Timestamp(v), (rc_Timestamp)v.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE),
                                     v.get_height(), v.get_width(), v.get_stride_in_bytes(),
                                     v.get_data(),
                                     [](void *f){ delete (rs2::frame*)f; }, (void*)new rs2::frame(frame)))
                    throw std::runtime_error((std::stringstream() << "failed receive for frame " << frame).str());
            } else if (s.format() == RS2_FORMAT_MOTION_XYZ32F) {
                const auto &m = frame.as<rs2::motion_frame>();
                const auto &p = s.as<rs2::motion_stream_profile>();
                const auto d = m.get_motion_data();
                if (s.stream_type() == RS2_STREAM_GYRO) {
                    if (!rc_receiveGyro(rc.get(), sensor_id[p.unique_id()], to_rc_Timestamp(m), rc_Vector{{ d.x, d.y, d.z }}))
                        throw std::runtime_error((std::stringstream() << "failed receive for frame " << frame).str());
                } else if (s.stream_type() == RS2_STREAM_ACCEL) {
                    if (!rc_receiveAccelerometer(rc.get(), sensor_id[p.unique_id()], to_rc_Timestamp(m), rc_Vector{{ d.x, d.y, d.z }}))
                        throw std::runtime_error((std::stringstream() << "failed receive for frame " << frame).str());
                }
            }
        }
        } catch (const rs2::error & e) {
            std::cerr << "callback failed " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "calback failed " << e.what() << std::endl;
        }
    });

    for(int s=0; s<3; s++)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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
