#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "example.hpp" // Include short list of convenience functions for rendering

// Struct for managing rotation of pointcloud view
struct state
{
    double yaw, pitch, last_x, last_y;
    bool ml;
    float offset_x, offset_y;
    texture tex;
};

// Helper functions
void register_glfw_callbacks(window &app, state &app_state);
void draw_pointcloud(window &app, state &app_state, rs2::points &points);

// Create a simple OpenGL window for rendering:
window app(1280, 720, "RealSense Pointcloud Example");
// Construct an object to manage view state
state app_state = {0, 0, 0, 0, false, 0, 0, 0};
// register callbacks to allow manipulation of the pointcloud
register_glfw_callbacks(app, app_state);

using namespace rs2;

// Declare pointcloud object, for calculating pointclouds and texture mappings
pointcloud pc = rs2::context().create_pointcloud();
// We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points points;

// Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline pipe;
// Start streaming with default recommended configuration
pipe.start();