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

struct point3d {
    float f[3];

    point3d() {}
    point3d(float x, float y, float z) : f{x, y, z} {}
    float x() const { return f[0]; }
    float y() const { return f[1]; }
    float z() const { return f[2]; }
};

struct pixel {
    float f[2];

    pixel() {}
    pixel(float x, float y) : f{x, y} {}
    float x() const { return f[0]; }
    float y() const { return f[1]; }
};

// We define a virtual object as a collection of vertices that will be connected by lines
typedef std::array<point3d, 4> object;

static rs2_pose identity_pose();
static rs2_pose pose_inverse(const rs2_pose& p);
static rs2_pose pose_multiply(const rs2_pose& ref2_in_ref1, const rs2_pose& ref3_in_ref2);
static rs2_quaternion quaternion_conjugate(const rs2_quaternion& q);
static rs2_quaternion quaternion_multiply(const rs2_quaternion& a, const rs2_quaternion& b);
static rs2_vector quaternion_rotate_vector(const rs2_quaternion& q, const rs2_vector& v);
static rs2_vector pose_transform_point(const rs2_pose& pose, const rs2_vector& p);
static rs2_vector vector_addition(const rs2_vector& a, const rs2_vector& b);
static rs2_vector vector_negate(const rs2_vector& v);

static std::vector<point3d> raster_line(const point3d& a, const point3d& b, float step);
static void render_line(const std::vector<pixel>& line, size_t color_code);
static void render_text(int x, int y, const std::string& text);
static void bin_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes);
static std::vector<uint8_t> bytes_from_bin_file(const std::string& filename);

class sensor_reprojection {
public:
    sensor_reprojection(rs2::pose_sensor &tm_sensor, const rs2_extrinsics &extrinsics) : _tm_sensor(tm_sensor), _extrinsics(extrinsics) {
        _object_pose_in_device.translation.x = 0;
        _object_pose_in_device.translation.y = 0;
        _object_pose_in_device.translation.z = -0.50; // 50 centimeter away in front of the camera.
        _object_pose_in_device.rotation = { 0, 0, 0, 1 };
    }

    std::vector<object> get_objects_in_sensor(const rs2_pose &device_pose_in_world, const std::map<std::string, rs2_pose> &objects_in_world)
    {
        std::vector<object> objects_in_sensor;
        rs2_pose world_pose_in_device = pose_inverse(device_pose_in_world);
        for (auto &each_object : objects_in_world) {
            // Compute the pose of the object relative to the current pose of the device
            rs2_pose w_object;
            if (_tm_sensor.get_static_node(each_object.first, w_object.translation, w_object.rotation)) {
                rs2_pose object_pose_in_device = pose_multiply(world_pose_in_device, w_object);

                // Get the object vertices in device coordinates
                object object_in_device = convert_object_coordinates(_virtual_object, object_pose_in_device);

                object object_in_sensor;
                // Convert object vertices from device coordinates into fisheye sensor coordinates using extrinsics
                for (size_t i = 0; i < object_in_device.size(); ++i)
                {
                    rs2_transform_point_to_point(object_in_sensor[i].f, &_extrinsics, object_in_device[i].f);
                }
                objects_in_sensor.emplace_back(object_in_sensor);
            }
        }
        return objects_in_sensor;
    }

    bool set_object_pose(rs2_pose &object_pose_in_world, const char *object_id, const rs2_pose& device_pose_in_world = identity_pose()) {
        // Convert the pose of the virtual object from camera coordinates into world coordinates
        rs2_pose _object_pose_in_world = pose_multiply(device_pose_in_world, _object_pose_in_device);
        if (object_id && _tm_sensor.set_static_node(object_id, _object_pose_in_world.translation, _object_pose_in_world.rotation)) {
            std::cout << "Setting new pose " << _object_pose_in_world.translation << " for virtual object: " << object_id << std::endl;
            object_pose_in_world = _object_pose_in_world;
            return true;
        }
        return false;
    }

private:
    rs2_extrinsics _extrinsics;
    rs2::pose_sensor _tm_sensor;
    // Create the vertices of a simple virtual object.
    // This virtual object is 4 points in 3D space that describe 3 XYZ 20cm long axes.
    // These vertices are relative to the object's own coordinate system.
    const float _length = 0.20f;
    const object _virtual_object = { {
        { 0, 0, 0 },      // origin
        { _length, 0, 0 }, // X
        { 0, _length, 0 }, // Y
        { 0, 0, _length }  // Z
    } };

    // Oject 50 centimeter away in front of the camera.
    // T265 coordinate system is defined here: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#sensor-origin-and-coordinate-system
    rs2_pose _object_pose_in_device;

    object convert_object_coordinates(const object& obj, const rs2_pose& object_pose)
    {
        object transformed_obj;
        for (size_t i = 0; i < obj.size(); ++i) {
            rs2_vector v{ obj[i].x(), obj[i].y(), obj[i].z() };
            v = pose_transform_point(object_pose, v);
            transformed_obj[i].f[0] = v.x;
            transformed_obj[i].f[1] = v.y;
            transformed_obj[i].f[2] = v.z;
        }
        return transformed_obj;
    }
};

int main(int argc, char * argv[]) try
{
    std::cout << "Waiting for T265 device..." << std::endl;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Enable fisheye and pose streams
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::pipeline_profile pipe_profile = cfg.resolve(pipe);
    // Initialize a shared pointer to a device with the current device on the pipeline
    rs2::pose_sensor tm_sensor = pipe_profile.get_device().first<rs2::pose_sensor>();
    const char *map_path = "map.bin";
    const bool load_map = false, save_map = true;
    if (tm_sensor && load_map) tm_sensor.import_localization_map(bytes_from_bin_file(map_path));

    // Start pipeline with chosen configuration
    pipe.start(cfg);
    // T265 has two fisheye sensors, we can choose any of them (index 1 or 2)
    const int fisheye_sensor_idx = 1;

    // Get fisheye sensor intrinsics parameters
    rs2::stream_profile fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();

    // Get fisheye sensor extrinsics parameters.
    // This is the pose of the fisheye sensor relative to the T265 coordinate system.
    sensor_reprojection projection_on_sensor(tm_sensor, fisheye_stream.get_extrinsics_to(pipe_profile.get_stream(RS2_STREAM_POSE)));

    std::cout << "Device got. Streaming data" << std::endl;

    // Create an OpenGL display window and a texture to draw the fisheye image
    window app(intrinsics.width, intrinsics.height, "Intel RealSense T265 Augmented Reality Example");
    window_key_listener key_watcher(app);
    texture fisheye_image;

    // This variable will hold the pose of the virtual object in world coordinates.
    // We we initialize it once we get the first pose frame.
    std::map<std::string, rs2_pose> objects_in_world;

    // Main loop
    while (app)
    {
        rs2_pose device_pose_in_world; // This will contain the current device pose
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            // Get a frame from the fisheye stream
            rs2::video_frame fisheye_frame = frames.get_fisheye_frame(fisheye_sensor_idx);
            // Get a frame from the pose stream
            rs2::pose_frame pose_frame = frames.get_pose_frame();

            // Copy current camera pose
            device_pose_in_world = pose_frame.get_pose_data();

            // Render the fisheye image
            fisheye_image.render(fisheye_frame, { 0, 0, app.width(), app.height() });

            // By closing the current scope we let frames be deallocated, so we do not fill up librealsense queues while we do other computation.
        }

        // Convert object vertices from device coordinates into fisheye sensor coordinates using extrinsics
        for (auto &each_object : projection_on_sensor.get_objects_in_sensor(device_pose_in_world, objects_in_world))
        {
            for (size_t i = 1; i < each_object.size(); ++i)
            {
                // Discretize the virtual object line into smaller 1cm long segments
                std::vector<point3d> points_in_sensor = raster_line(each_object[0], each_object[i], 0.01f);
                std::vector<pixel> projected_line;
                projected_line.reserve(points_in_sensor.size());
                for (auto& point : points_in_sensor)
                {
                    // A 3D point is visible in the image if its Z coordinate relative to the fisheye sensor is positive.
                    if (point.z() > 0)
                    {
                        // Project 3D sensor coordinates to 2D fisheye image coordinates using intrinsics
                        projected_line.emplace_back();
                        rs2_project_point_to_pixel(projected_line.back().f, &intrinsics, point.f);
                    }
                }
                // Display the line in the image
                render_line(projected_line, i);
            }
            pixel origin;
            rs2_project_point_to_pixel(origin.f, &intrinsics, each_object[0].f);
            render_text(origin.x(), origin.y(), "T");
        }

        // Display text in the image
        render_text(15, (app.height() - 10) / 2, "Press spacebar to reset the pose of the virtual object. Press ESC to exit");

        // Check if some key is pressed
        switch (key_watcher.get_key())
        {
        case GLFW_KEY_SPACE: {
            // Set/Reset virtual object pose if user presses spacebar
            rs2_pose object_pose_in_world;
            char *node_name = "1";
            if (objects_in_world.size() >= 1) node_name = "2";
            if (projection_on_sensor.set_object_pose(object_pose_in_world, node_name, device_pose_in_world))
                objects_in_world[node_name] = object_pose_in_world;
            break;
        }
        case GLFW_KEY_S:
            if (save_map)
            {
                pipe.stop();
                bin_file_from_bytes(map_path, tm_sensor.export_localization_map());
                std::cout << "Saving relocalization map to: " << map_path << std::endl;
            }
            //break;
        case GLFW_KEY_ESCAPE:
            // Exit if user presses escape
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

rs2_pose identity_pose()
{
    // Return an identity pose (no translation, no rotation)
    rs2_pose pose;
    pose.translation.x = 0;
    pose.translation.y = 0;
    pose.translation.z = 0;
    pose.rotation.x = 0;
    pose.rotation.y = 0;
    pose.rotation.z = 0;
    pose.rotation.w = 1;
    return pose;
}

rs2_pose pose_inverse(const rs2_pose& p)
{
    rs2_pose i;
    i.rotation = quaternion_conjugate(p.rotation);
    i.translation = vector_negate(quaternion_rotate_vector(i.rotation, p.translation));
    return i;
}

rs2_pose pose_multiply(const rs2_pose& ref2_in_ref1, const rs2_pose& ref3_in_ref2)
{
    rs2_pose ref3_in_ref1;
    ref3_in_ref1.rotation = quaternion_multiply(ref2_in_ref1.rotation, ref3_in_ref2.rotation);
    ref3_in_ref1.translation = vector_addition(quaternion_rotate_vector(ref2_in_ref1.rotation, ref3_in_ref2.translation), ref2_in_ref1.translation);
    return ref3_in_ref1;
}

rs2_vector pose_transform_point(const rs2_pose& pose, const rs2_vector& p)
{
    return vector_addition(quaternion_rotate_vector(pose.rotation, p), pose.translation);
}

rs2_quaternion quaternion_multiply(const rs2_quaternion& a, const rs2_quaternion& b)
{
    return rs2_quaternion {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

rs2_vector quaternion_rotate_vector(const rs2_quaternion& q, const rs2_vector& v)
{
    rs2_quaternion v_as_quaternion = { v.x, v.y, v.z, 0 };
    rs2_quaternion rotated_v = quaternion_multiply(quaternion_multiply(q, v_as_quaternion), quaternion_conjugate(q));
    return rs2_vector { rotated_v.x, rotated_v.y, rotated_v.z };
}

rs2_quaternion quaternion_conjugate(const rs2_quaternion& q)
{
    return rs2_quaternion { -q.x, -q.y, -q.z, q.w };
}

rs2_vector vector_addition(const rs2_vector& a, const rs2_vector& b)
{
    return rs2_vector { a.x + b.x, a.y + b.y, a.z + b.z };
}

rs2_vector vector_negate(const rs2_vector& v)
{
    return rs2_vector { -v.x, -v.y, -v.z };
}

std::vector<point3d> raster_line(const point3d& a, const point3d& b, float step)
{
    rs2_vector direction = { b.x() - a.x(), b.y() - a.y(), b.z() - a.z() };
    float distance = std::sqrt(direction.x*direction.x + direction.y*direction.y + direction.z*direction.z);
    int npoints = distance / step + 1;

    std::vector<point3d> points;
    if (npoints > 0)
    {
        direction.x = direction.x * step / distance;
        direction.y = direction.y * step / distance;
        direction.z = direction.z * step / distance;

        points.reserve(npoints);
        points.emplace_back(a);
        for (int i = 1; i < npoints; ++i)
        {
            points.emplace_back(a.x() + direction.x * i,
                                a.y() + direction.y * i,
                                a.z() + direction.z * i);
        }
    }
    return points;
}

void render_line(const std::vector<pixel>& line, size_t color_code)
{
    if (!line.empty())
    {
        GLfloat current_color[4];
        glGetFloatv(GL_CURRENT_COLOR, current_color);

        glLineWidth(5);
        glColor3f(color_code == 1 ? 1.f : 0.f,
                  color_code == 2 ? 1.f : 0.f,
                  color_code == 3 ? 1.f : 0.f);

        glBegin(GL_LINE_STRIP);
        for (auto& pixel : line)
        {
            glVertex3f(pixel.x(), pixel.y(), 0.f);
        }
        glEnd();

        glColor4fv(current_color);
    }
}

void render_text(int x, int y, const std::string& text)
{
    GLfloat current_color[4];
    glGetFloatv(GL_CURRENT_COLOR, current_color);
    glColor3f(0, 0.5, 1);
    glScalef(2, 2, 2);
    draw_text(x, y, text.c_str());
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
