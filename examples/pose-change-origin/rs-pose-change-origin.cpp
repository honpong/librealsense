// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "example.hpp"

static void render_text(int win_height, const std::string& text);

int main(int argc, char * argv[]) try
{
    std::cout << "Waiting for device..." << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Enable fisheye and pose streams
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    auto pose_sensor = pipe_profile.get_device().first<rs2::pose_sensor>();

    std::string node_name;
    int node_num = 0;

    // Create an OpenGL display window and a texture to draw the fisheye image
    window app(1280, 720, "Intel RealSense T265 Augmented Reality Example");
    window_key_listener key_watcher(app);
    texture fisheye_image;


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
            double effectiveTime;
            pose_sensor.change_pose_origin(node_name, effectiveTime);
            std::cout << std::endl;
            std::cout << "Effective Time: " << effectiveTime << std::endl;;
            break;
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
            pipe.stop();
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
