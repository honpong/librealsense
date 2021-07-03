// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include "example.hpp"

//#include "tclap/CmdLine.h"

static void render_text(int win_height, const std::string& text);
static void bin_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes);
static std::vector<uint8_t> bytes_from_bin_file(const std::string& filename);

int main(int argc, char * argv[]) try
{
    const char *save_map = nullptr, *load_map = nullptr;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--save-map") == 0 && i + 1 < argc) { save_map = argv[++i]; }
        else if (strcmp(argv[i], "--load-map") == 0 && i + 1 < argc) { load_map = argv[++i]; }
    }

    std::cout << "Waiting for device..." << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Enable pose streams
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    rs2::pipeline_profile pipe_profile = cfg.resolve(pipe);
    // Initialize a shared pointer to a device with the current device on the pipeline
    rs2::pose_sensor pose_sensor = pipe_profile.get_device().first<rs2::pose_sensor>();

    if (pose_sensor && load_map) {
        if (pose_sensor.import_localization_map(bytes_from_bin_file(load_map))) {
            std::cout << "Map loading success." << std::endl;
        }
    }
    bool is_map_relocalized = false;
    pose_sensor.set_notifications_callback([&](const rs2::notification& n) {
        if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
            std::cout << "Relocalization Event Detected." << std::endl;
            is_map_relocalized = true;
            //}
        }
        });

    // Start pipeline with chosen configuration
    pipe_profile = pipe.start(cfg);

    std::string node_name;
    int node_num = 0;
    double effectiveTime;

    // Create an OpenGL display window and a texture to draw the fisheye image
    window app(1280, 720, "Intel RealSense T265 Augmented Reality Example");
    window_key_listener key_watcher(app);

    // Main loop
    while (app)
    {

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
            pose_data.translation.y << " " << pose_data.translation.z << " " <<
            pose_data.rotation.x << " " << pose_data.rotation.y << " " <<
            pose_data.rotation.z << " " << pose_data.rotation.w;

        render_text(app.height(), "Press 'S' to set static node, 'C' to change origin");

        switch (key_watcher.get_key())
        {
        case GLFW_KEY_S:
            node_name = "node" + std::to_string(node_num++);
            pose_sensor.set_static_node(node_name,
                pose_data.translation,
                pose_data.rotation);
            break;
        case GLFW_KEY_C:
            pose_sensor.set_pose_origin(node_name, effectiveTime);
            std::cout << std::endl;
            std::cout << "Change pose to node: Effective Time: " << effectiveTime << std::endl;
            break;
        case GLFW_KEY_0:
            pose_sensor.set_pose_origin(0, effectiveTime);
            std::cout << std::endl;
            std::cout << "Change pose to map ID 0: Effective Time: " << effectiveTime << std::endl;
            break;
        case GLFW_KEY_1:
            if (is_map_relocalized)
            {
                pose_sensor.set_pose_origin(1, effectiveTime);
                std::cout << std::endl;
                std::cout << "Change pose to map ID 1: Effective Time: " << effectiveTime << std::endl;
            }
            break;
        case GLFW_KEY_E:
            if (save_map) {
                bin_file_from_bytes(save_map, pose_sensor.export_localization_map());
                std::cout << "Saving relocalization map to: " << save_map << std::endl;
            }
            break;
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
            pipe.stop();
            if (save_map) {
                bin_file_from_bytes(save_map, pose_sensor.export_localization_map());
                std::cout << "Saving relocalization map to: " << save_map << std::endl;
            }
            app.close();
            break;
        }
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

void render_text(int win_height, const std::string& text)
{
    GLfloat current_color[4];
    glGetFloatv(GL_CURRENT_COLOR, current_color);
    glColor3f(0, 0.5, 1);
    glScalef(2, 2, 2);
    draw_text(15, (win_height - 10) / 2, text.c_str());
    glScalef(1, 1, 1);
    glColor4fv(current_color);
}

void bin_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
{
    std::ofstream file(filename, std::ios::binary | std::ios::trunc);
    if (!file.good())
        throw std::runtime_error("Invalid binary file specified. Verify the target path and location permissions");
    file.write((char*)bytes.data(), bytes.size());
}

std::vector<uint8_t> bytes_from_bin_file(const std::string& filename)
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
