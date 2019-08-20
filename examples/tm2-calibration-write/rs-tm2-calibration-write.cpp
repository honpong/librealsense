// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <string.h>

using namespace std;

std::vector<std::string> tokenize_floats(string input, char separator) {
    std::vector<std::string> tokens;
    stringstream ss(input);
    string token;

    while (std::getline(ss, token, separator)) {
        tokens.push_back(token);
    }

    return tokens;
}

void print(const rs2_extrinsics& extrinsics)
{
    stringstream ss;
    ss << " Rotation Matrix:\n";

    // Align displayed data along decimal point
    for (auto i = 0; i < 3; ++i)
    {
        for (auto j = 0; j < 3; ++j)
        {
            std::ostringstream oss;
            oss << extrinsics.rotation[j * 3 + i];
            auto tokens = tokenize_floats(oss.str().c_str(), '.');
            ss << right << setw(4) << tokens[0];
            if (tokens.size() > 1)
                ss << "." << left << setw(12) << tokens[1];
        }
        ss << endl;
    }

    ss << "\n Translation Vector: ";
    for (auto i = 0u; i < sizeof(extrinsics.translation) / sizeof(extrinsics.translation[0]); ++i)
        ss << setprecision(15) << extrinsics.translation[i] << "  ";

    cout << ss.str() << endl << endl;
}

void print(const rs2_motion_device_intrinsic& intrinsics)
{
    stringstream ss;
    ss << "Bias Variances: \t";

    for (auto i = 0u; i < sizeof(intrinsics.bias_variances) / sizeof(intrinsics.bias_variances[0]); ++i)
        ss << setprecision(15) << std::fixed << intrinsics.bias_variances[i] << "  ";

    ss << "\nNoise Variances: \t";
    for (auto i = 0u; i < sizeof(intrinsics.noise_variances) / sizeof(intrinsics.noise_variances[0]); ++i)
        ss << setprecision(15) << std::fixed << intrinsics.noise_variances[i] << "  ";

    ss << "\nSensitivity : " << std::endl;
    for (auto i = 0u; i < sizeof(intrinsics.data) / sizeof(intrinsics.data[0]); ++i)
    {
        for (auto j = 0u; j < sizeof(intrinsics.data[0]) / sizeof(intrinsics.data[0][0]); ++j)
            ss << std::right << std::setw(13) << setprecision(6) << intrinsics.data[i][j] << "  ";
        ss << "\n";
    }

    cout << ss.str() << endl << endl;
}

void print(const rs2_intrinsics& intrinsics)
{
    stringstream ss;
    ss << left << setw(14) << "  Width: " << "\t" << intrinsics.width << endl <<
        left << setw(14) << "  Height: " << "\t" << intrinsics.height << endl <<
        left << setw(14) << "  PPX: " << "\t" << setprecision(15) << intrinsics.ppx << endl <<
        left << setw(14) << "  PPY: " << "\t" << setprecision(15) << intrinsics.ppy << endl <<
        left << setw(14) << "  Fx: " << "\t" << setprecision(15) << intrinsics.fx << endl <<
        left << setw(14) << "  Fy: " << "\t" << setprecision(15) << intrinsics.fy << endl <<
        left << setw(14) << "  Distortion: " << "\t" << rs2_distortion_to_string(intrinsics.model) << endl <<
        left << setw(14) << "  Coeffs: ";

    for (auto i = 0u; i < sizeof(intrinsics.coeffs) / sizeof(intrinsics.coeffs[0]); ++i)
        ss << "\t" << setprecision(15) << intrinsics.coeffs[i] << "  ";

    cout << ss.str() << endl << endl;
}

bool test_write_oem_calibration()
{
    rs2::context ctx;
    for (auto dev : ctx.query_devices()) {
        auto name = dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
        if (strcmp(name, "Intel RealSense T265") == 0) {
            auto tm2 = dev.as<rs2::tm2>();
            rs2_intrinsics fisheye_intrinsics[2] = {};
            rs2_motion_device_intrinsic motion_intrinsics[2] = {};

            // set calibration data into the device's virtual table
            for (auto sensor_id : { 1,2 }) {
                tm2.set_intrinsics(sensor_id, fisheye_intrinsics[sensor_id - 1]);
            }
            for (auto imu : { RS2_STREAM_GYRO, RS2_STREAM_ACCEL }) {
                tm2.set_motion_device_intrinsics(imu, motion_intrinsics[imu - RS2_STREAM_GYRO]);
            }
            tm2.write_calibration(); //write new calibration data into EEPROM
        }
        return true;
    }
    return false;
}

void test_read_oem_calibration()
{
    rs2::context ctx;
    for (auto dev : ctx.query_devices()) {
        auto name = dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
        if (strcmp(name, "Intel RealSense T265") == 0) {
            auto tm2 = dev.as<rs2::tm2>();
            for (auto sensor : tm2.query_sensors()) {
                for (auto profile : sensor.get_stream_profiles()) {
                    // read camera intrinsics calibration
                    if (auto vp = profile.as<rs2::video_stream_profile>()) {
                        print(vp.get_intrinsics());
                    }
                    // read imu intrinsics calibration
                    if (auto mp = profile.as<rs2::motion_stream_profile>()) {
                        print(mp.get_motion_intrinsics());
                    }
                }
            }
            tm2.reset_to_factory_calibration(); // reset to factory calibration
        }
    }
}

int main(int argc, char * argv[]) try
{
    if (test_write_oem_calibration() == true) {
        test_read_oem_calibration();
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
